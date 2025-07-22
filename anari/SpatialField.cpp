// Copyright 2023 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

// std
#include <cfloat>
// ours
#include "Array.h"
#include "SpatialField.h"

namespace barney_device {

SpatialField::SpatialField(BarneyGlobalState *s)
    : Object(ANARI_SPATIAL_FIELD, s)
{}

SpatialField::~SpatialField() = default;

SpatialField *SpatialField::createInstance(
    std::string_view subtype, BarneyGlobalState *s)
{
  if (subtype == "unstructured")
    return new UnstructuredField(s);
  else if (subtype == "amr")
    return new BlockStructuredField(s);
  else if (subtype == "structuredRegular")
    return new StructuredRegularField(s);
  else if (subtype == "planet")
    return new PlanetSpatialField(s);
  else if (subtype == "cloud")
    return new CloudSpatialField(s);
  else
    return (SpatialField *)new UnknownObject(ANARI_SPATIAL_FIELD, s);
}

void SpatialField::markFinalized()
{
  deviceState()->markSceneChanged();
  Object::markFinalized();
}

// Subtypes ///////////////////////////////////////////////////////////////////

// StructuredRegularField //

StructuredRegularField::StructuredRegularField(BarneyGlobalState *s)
    : SpatialField(s)
{}

void StructuredRegularField::commitParameters()
{
  Object::commitParameters();
  m_data = getParamObject<helium::Array3D>("data");
  m_origin = getParam<helium::float3>("origin", helium::float3(0.f));
  m_spacing = getParam<helium::float3>("spacing", helium::float3(1.f));
  m_dims = m_data->size();
}

void StructuredRegularField::finalize()
{
  if (!m_data) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'data' on 'structuredRegular' field");
    return;
  }

  const auto dims = m_data->size();
  m_coordUpperBound = helium::float3(std::nextafterf((float)dims.x - 1, 0),
      std::nextafterf((float)dims.y - 1, 0),
      std::nextafterf((float)dims.z - 1, 0));
}

bool StructuredRegularField::isValid() const
{
  return m_data;
}

BNScalarField StructuredRegularField::createBarneyScalarField() const
{
  if (!isValid())
    return {};

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNDataType barneyType;
  switch (m_data->elementType()) {
  case ANARI_FLOAT32:
    barneyType = BN_FLOAT;
    break;
  case ANARI_UFIXED8:
  case ANARI_UINT8:
    barneyType = BN_UFIXED8;
    break;
  default:
    throw std::runtime_error("scalar type not implemented ...");
  }
  auto dims = m_data->size();

  BNScalarField sf = bnScalarFieldCreate(context, slot, "structured");
#if 1
  BNTextureData td = bnTextureData3DCreate(
      context, slot, barneyType, dims.x, dims.y, dims.z, m_data->data());
  bnSetObject(sf, "textureData", td);
  bnRelease(td);
#else
  BNTexture3D texture = bnTexture3DCreate(context,
      slot,
      barneyType,
      dims.x,
      dims.y,
      dims.z,
      m_data->data(),
      BN_TEXTURE_LINEAR,
      BN_TEXTURE_CLAMP);
  bnSetObject(sf, "texture", texture);
  bnRelease(texture);
#endif
  bnSet3i(sf, "dims", dims.x, dims.y, dims.z);
  bnSet3fc(sf, "gridOrigin", m_origin);
  bnSet3fc(sf, "gridSpacing", m_spacing);
  bnCommit(sf);
  auto field = sf;

  return field;
}

box3 StructuredRegularField::bounds() const
{
  return isValid()
      ? box3(m_origin, m_origin + ((helium::float3(m_dims) - 1.f) * m_spacing))
      : box3{};
}

// UnstructuredField //

UnstructuredField::UnstructuredField(BarneyGlobalState *s) : SpatialField(s) {}

void UnstructuredField::commitParameters()
{
  Object::commitParameters();

  m_params.vertexPosition = getParamObject<helium::Array1D>("vertex.position");
  m_params.vertexData = getParamObject<helium::Array1D>("vertex.data");
  m_params.cellData = getParamObject<helium::Array1D>("cell.data");
  m_params.index = getParamObject<helium::Array1D>("index");
  m_params.cellType = getParamObject<helium::Array1D>("cell.type");
  m_params.cellBegin = getParamObject<helium::Array1D>("cell.begin");
  if (!m_params.cellBegin) // some older apps use "cell.index"
    m_params.cellBegin = getParamObject<helium::Array1D>("cell.index");
}

void UnstructuredField::finalize()
{
  if (!m_params.vertexPosition) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'vertex.position' "
        "on unstructured spatial field");
    return;
  }
  if (m_params.vertexPosition->elementType() != ANARI_FLOAT32_VEC3) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "'unstructured::vertex.position' must be ANARI_FLOAT32_VEC3 (is %i) ",
        m_params.vertexPosition->elementType());
    return;
  }

  if (!m_params.vertexData && !m_params.cellData) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'vertex.data' OR"
        " 'cell.data' on unstructured spatial field");
    return;
  }

  if (m_params.vertexData && m_params.cellData) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "cannot have both 'cell.data' and 'vertex.data' "
        "on unstructured spatial field");
    return;
  }

  if (!m_params.index) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'index' on unstructured spatial field");
    return;
  }

  if (!m_params.cellType) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'cell.type' on unstructured spatial field");
    return;
  }

  if (!m_params.cellBegin) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'cell.begin' on unstructured spatial field");
    return;
  }

  m_bounds.invalidate();

  auto *vertexPositions = m_params.vertexPosition->beginAs<math::float3>();
  int numVertices = (int)m_params.vertexPosition->size();
  auto *vertexData =
      m_params.vertexData ? m_params.vertexData->beginAs<float>() : nullptr;
  auto *cellData =
      m_params.cellData ? m_params.cellData->beginAs<float>() : nullptr;
  int numScalars =
      cellData ? m_params.cellData->size() : m_params.vertexData->size();

  for (int i = 0; i < numVertices; i++)
    m_bounds.insert(vertexPositions[i]);

  uint32_t *index32{nullptr};
  // uint64_t *index64{nullptr};
  if (m_params.index->elementType() == ANARI_UINT32)
    index32 = (uint32_t *)m_params.index->beginAs<uint32_t>();
  else if (m_params.index->elementType() == ANARI_UINT64)
    //      index64 = (uint64_t *)m_params.index->beginAs<uint64_t>();
    reportMessage(ANARI_SEVERITY_ERROR,
        "'unstructured:index' - we only support 32-bit indices");
  else {
    reportMessage(ANARI_SEVERITY_ERROR,
        "parameter 'index' on unstructured spatial field has wrong element type");
    return;
  }
  if (m_params.cellBegin->elementType() == ANARI_UINT32)
    index32 = (uint32_t *)m_params.index->beginAs<uint32_t>();
  else if (m_params.cellBegin->elementType() == ANARI_UINT64)
    //      index64 = (uint64_t *)m_params.index->beginAs<uint64_t>();
    reportMessage(ANARI_SEVERITY_ERROR,
        "'unstructured:index' - we only support 32-bit indices");
  else {
    reportMessage(ANARI_SEVERITY_ERROR,
        "parameter 'index' on unstructured spatial field has wrong element type");
    return;
  }
}

BNScalarField UnstructuredField::createBarneyScalarField() const
{
  std::cout
      << "=================================================================="
      << std::endl;
  std::cout << "BANARI: CREATING UMESH OF " << m_params.cellBegin->size()
            << " elements" << std::endl;
  std::cout
      << "=================================================================="
      << std::endl;

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  auto *vertexPositions = m_params.vertexPosition->beginAs<math::float3>();
  int numVertices = (int)m_params.vertexPosition->size();
  // endAs<math::float3>()
  //   - m_params.vertexPosition->beginAs<math::float3>();

  auto *vertexData =
      m_params.vertexData ? m_params.vertexData->beginAs<float>() : nullptr;
  auto *cellData =
      m_params.cellData ? m_params.cellData->beginAs<float>() : nullptr;
  assert(vertexData || cellData);
  int numScalars =
      (int)(cellData ? m_params.cellData->size() : m_params.vertexData->size());

  BNData verticesData =
      bnDataCreate(context, slot, BN_FLOAT3, numVertices, vertexPositions);
  BNData scalarsData = bnDataCreate(
      context, slot, BN_FLOAT, numScalars, vertexData ? vertexData : cellData);
  BNData indicesData = bnDataCreate(context,
      slot,
      BN_INT,
      m_params.index->size(),
      (const int *)m_params.index->data());
  BNData cellTypeData = bnDataCreate(context,
      slot,
      BN_UINT8,
      m_params.cellType->size(),
      (const int *)m_params.cellType->data());
  BNData elementOffsetsData = bnDataCreate(context,
      slot,
      BN_INT,
      m_params.cellBegin->size(),
      (const int *)m_params.cellBegin->data());
  BNScalarField sf = bnScalarFieldCreate(context, slot, "unstructured");
  bnSetData(sf, "vertex.position", verticesData);
  if (vertexData) {
    // this will atomatically set cell.data to 0 on barney side
    bnSetData(sf, "vertex.data", scalarsData);
  } else {
    // this will atomatically set vertex.data to 0 on barney side
    bnSetData(sf, "cell.data", scalarsData);
  }
  bnSetData(sf, "index", indicesData);
  bnSetData(sf, "cell.index", elementOffsetsData);
  bnSetData(sf, "cell.type", cellTypeData);
  bnCommit(sf);
  return sf;
}

box3 UnstructuredField::bounds() const
{
  std::cout << "BANARI: BOUNDS OF UNSTRUCTURED FIELD" << std::endl;
  return m_bounds;
}

// BlockStructuredField //

BlockStructuredField::BlockStructuredField(BarneyGlobalState *s)
    : SpatialField(s)
{}

void BlockStructuredField::commitParameters()
{
  Object::commitParameters();
  m_params.cellWidth = getParamObject<helium::Array1D>("cellWidth");
  m_params.blockBounds = getParamObject<helium::Array1D>("block.bounds");
  m_params.blockLevel = getParamObject<helium::Array1D>("block.level");
  m_params.blockData = getParamObject<helium::ObjectArray>("block.data");
}

void BlockStructuredField::finalize()
{
  if (!m_params.blockBounds) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'block.bounds' on amr spatial field");
    return;
  }

  if (!m_params.blockLevel) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'block.level' on amr spatial field");
    return;
  }

  if (!m_params.blockData) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'block.data' on amr spatial field");
    return;
  }

  size_t numBlocks = m_params.blockData->totalSize();
  auto *blockBounds = m_params.blockBounds->beginAs<box3i>();
  auto *blockLevels = m_params.blockLevel->beginAs<int>();
  auto *blockData = (helium::Array3D **)m_params.blockData->handlesBegin();

  m_generatedBlockBounds.clear();
  m_generatedBlockLevels.clear();
  m_generatedBlockOffsets.clear();
  m_generatedBlockScalars.clear();

  m_bounds.invalidate();

  for (size_t i = 0; i < numBlocks; ++i) {
    const box3i bounds = *(blockBounds + i);
    const int level = *(blockLevels + i);
    const helium::Array3D *bd = *(blockData + i);

    m_generatedBlockBounds.push_back(bounds.lower.x);
    m_generatedBlockBounds.push_back(bounds.lower.y);
    m_generatedBlockBounds.push_back(bounds.lower.z);
    m_generatedBlockBounds.push_back(bounds.upper.x);
    m_generatedBlockBounds.push_back(bounds.upper.y);
    m_generatedBlockBounds.push_back(bounds.upper.z);
    m_generatedBlockLevels.push_back(level);
    m_generatedBlockOffsets.push_back((int)m_generatedBlockScalars.size());

    for (unsigned z = 0; z < bd->size().z; ++z)
      for (unsigned y = 0; y < bd->size().y; ++y)
        for (unsigned x = 0; x < bd->size().x; ++x) {
          size_t index =
              z * size_t(bd->size().x) * bd->size().y + y * bd->size().x + x;
          float f = bd->dataAs<float>()[index];
          m_generatedBlockScalars.push_back(f);
        }

    box3 worldBounds;
    worldBounds.lower = math::float3(float(bounds.lower.x * (1 << level)),
        float(bounds.lower.y * (1 << level)),
        float(bounds.lower.z * (1 << level)));
    worldBounds.upper = math::float3(float((bounds.upper.x + 1) * (1 << level)),
        float((bounds.upper.y + 1) * (1 << level)),
        float((bounds.upper.z + 1) * (1 << level)));
    m_bounds.insert(worldBounds);
  }
}

BNScalarField BlockStructuredField::createBarneyScalarField() const
{
  std::cout
      << "=================================================================="
      << std::endl;
  std::cout << "BANARI: CREATING AMR DATA" << std::endl;
  std::cout
      << "=================================================================="
      << std::endl;
#if 1
  exit(0);
#else
  return bnBlockStructuredAMRCreate(context,
      slot,
      m_generatedBlockBounds.data(),
      m_generatedBlockBounds.size() / 6,
      m_generatedBlockLevels.data(),
      m_generatedBlockOffsets.data(),
      m_generatedBlockScalars.data(),
      m_generatedBlockScalars.size());
#endif
}

box3 BlockStructuredField::bounds() const
{
  return m_bounds;
}

// PlanetSpatialField //

PlanetSpatialField::PlanetSpatialField(BarneyGlobalState *s) : SpatialField(s) {}

void PlanetSpatialField::commitParameters()
{
  Object::commitParameters();
  m_elevationMap = getParamObject<helium::Array2D>("elevationMap");
  m_diffuseMap = getParamObject<helium::Array2D>("diffuseMap");
  m_normalMap = getParamObject<helium::Array2D>("normalMap");
  
  m_sphereRadius = getParam<float>("sphereRadius", 0.5f);
  m_elevationScale = getParam<float>("elevationScale", 0.1f);
  m_atmosphereThickness = getParam<float>("atmosphereThickness", 0.2f);
  m_sphereCenter = getParam<helium::float3>("sphereCenter", helium::float3(0.f));
}

void PlanetSpatialField::finalize()
{
  // Planet field is always valid - it doesn't require external data
}

bool PlanetSpatialField::isValid() const
{
  return true; // Planet field provides default layered structure
}

BNScalarField PlanetSpatialField::createBarneyScalarField() const
{
  if (!isValid())
    return {};

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = bnScalarFieldCreate(context, slot, "planet");
  
  // Set planet parameters
  bnSet1f(sf, "sphereRadius", m_sphereRadius);
  bnSet1f(sf, "elevationScale", m_elevationScale);
  bnSet1f(sf, "atmosphereThickness", m_atmosphereThickness);
  bnSet3fc(sf, "sphereCenter", m_sphereCenter);
  
  // Set texture maps if provided
  if (m_elevationMap) {
    BNTextureData td = bnTextureData2DCreate(
        context, slot, BN_FLOAT32,
        m_elevationMap->size().x, m_elevationMap->size().y, 
        m_elevationMap->data());
    bnSetObject(sf, "elevationMap", td);
    bnRelease(td);
  }
  
  if (m_diffuseMap) {
    BNTextureData td = bnTextureData2DCreate(
        context, slot, BN_FLOAT32_VEC3,
        m_diffuseMap->size().x, m_diffuseMap->size().y,
        m_diffuseMap->data());
    bnSetObject(sf, "diffuseMap", td);
    bnRelease(td);
  }
  
  if (m_normalMap) {
    BNTextureData td = bnTextureData2DCreate(
        context, slot, BN_FLOAT32_VEC4,
        m_normalMap->size().x, m_normalMap->size().y,
        m_normalMap->data());
    bnSetObject(sf, "normalMap", td);
    bnRelease(td);
  }
  
  bnCommit(sf);
  return sf;
}

box3 PlanetSpatialField::bounds() const
{
  float totalRadius = m_sphereRadius + m_atmosphereThickness;
  helium::float3 center = m_sphereCenter;
  return box3(center - totalRadius, center + totalRadius);
}

// CloudSpatialField //

CloudSpatialField::CloudSpatialField(BarneyGlobalState *s) : SpatialField(s) {}

void CloudSpatialField::commitParameters()
{
  Object::commitParameters();
  
  // Store previous values to detect changes
  auto prevCloudData = m_cloudData;
  float prevPlanetRadius = m_planetRadius;
  float prevAtmosphereThickness = m_atmosphereThickness;
  
  m_cloudData = getParamObject<helium::Array3D>("cloudData");
  
  m_planetRadius = getParam<float>("planetRadius", DEFAULT_PLANET_RADIUS);
  m_atmosphereThickness = getParam<float>("atmosphereThickness", DEFAULT_ATMOSPHERE_THICKNESS);
  
  // Invalidate cached Barney scalar field if parameters changed
  if (prevCloudData != m_cloudData || 
      prevPlanetRadius != m_planetRadius || 
      prevAtmosphereThickness != m_atmosphereThickness) {
    cleanup();
  }
}

void CloudSpatialField::finalize()
{
  // Cloud field is always valid - it can work without external data
}

bool CloudSpatialField::isValid() const
{
  return true; // Cloud field provides default behavior even without cloudData
}

BNScalarField CloudSpatialField::createBarneyScalarField() const
{
  if (!isValid())
    return {};

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = bnScalarFieldCreate(context, slot, "cloud");
  
  // Set cloud parameters
  bnSet1f(sf, "planetRadius", m_planetRadius);
  bnSet1f(sf, "atmosphereThickness", m_atmosphereThickness);
  
  // Set cloud data texture if provided
  if (m_cloudData) {
    BNTextureData td = bnTextureData3DCreate(
        context, slot, BN_FLOAT32,
        m_cloudData->size().x, m_cloudData->size().y, m_cloudData->size().z,
        m_cloudData->data());
    bnSetObject(sf, "cloudData", td);
    bnRelease(td);
  }
  
  bnCommit(sf);
  return sf;
}

box3 CloudSpatialField::bounds() const
{
  float totalRadius = m_planetRadius + m_atmosphereThickness;
  return box3(-math::float3(totalRadius), math::float3(totalRadius));
}

} // namespace barney_device

BARNEY_ANARI_TYPEFOR_DEFINITION(barney_device::SpatialField *);
