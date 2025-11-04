// Copyright 2023 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

// std
#include <cfloat>
#include <numeric>
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
  else if (subtype == "clouds")
    return new CloudSpatialField(s);
  else if (subtype == "magnetic")
    return new MagneticSpatialField(s);
  else if (subtype == "aurora")
    return new AuroraSpatialField(s);
  else
    return (SpatialField *)new UnknownObject(ANARI_SPATIAL_FIELD, subtype, s);
}

void SpatialField::markFinalized()
{
  deviceState()->markSceneChanged();
  Object::markFinalized();
}

// Subtypes ///////////////////////////////////////////////////////////////////

// StructuredRegularField //

StructuredRegularField::StructuredRegularField(BarneyGlobalState *s)
    : SpatialField(s), m_data(this)
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

  //=======================================================
  // get (or create) and populate bn field
  //=======================================================

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = getBarneyScalarField();

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

  BNScalarField sf = bnScalarFieldCreate(context, slot, "structured");
  return sf;
}

box3 StructuredRegularField::bounds() const
{
  return isValid()
      ? box3(m_origin, m_origin + ((helium::float3(m_dims) - 1.f) * m_spacing))
      : box3{};
}

// UnstructuredField //

UnstructuredField::UnstructuredField(BarneyGlobalState *s)
    : SpatialField(s), m_params(this) {}

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
    int(cellData ? m_params.cellData->size() : m_params.vertexData->size());

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

  //=======================================================
  // get (or create) and populate bn field
  //=======================================================

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = getBarneyScalarField();

  if (!m_bnData.vertices) {
    m_bnData.vertices =
      bnDataCreate(context, slot, BN_FLOAT3, numVertices, vertexPositions);
  } else {
    bnDataSet(m_bnData.vertices, numVertices, vertexPositions);
  }

  if (!m_bnData.scalars) {
    m_bnData.scalars = bnDataCreate(
      context, slot, BN_FLOAT, numScalars, vertexData ? vertexData : cellData);
  } else {
    bnDataSet(m_bnData.scalars, numScalars, vertexData ? vertexData : cellData);
  }

  if (!m_bnData.indices) {
    m_bnData.indices = bnDataCreate(context,
      slot,
      BN_INT,
      m_params.index->size(),
      (const int *)m_params.index->data());
  } else {
    bnDataSet(m_bnData.indices, m_params.index->size(), (const int *)m_params.index->data());
  }

  if (!m_bnData.cellType) {
    m_bnData.cellType = bnDataCreate(context,
        slot,
        BN_UINT8,
        m_params.cellType->size(),
        (const int *)m_params.cellType->data());
  } else {
    bnDataSet(m_bnData.cellType, m_params.cellType->size(), (const int *)m_params.cellType->data());
  }

  if (!m_bnData.elementOffsets) {
    m_bnData.elementOffsets = bnDataCreate(context,
      slot,
      BN_INT,
      m_params.cellBegin->size(),
      (const int *)m_params.cellBegin->data());
  } else {
    bnDataSet(m_bnData.elementOffsets, m_params.cellBegin->size(), (const int *)m_params.cellBegin->data());
  }

  bnSetData(sf, "vertex.position", m_bnData.vertices);
  if (vertexData) {
    // this will atomatically set cell.data to 0 on barney side
    bnSetData(sf, "vertex.data", m_bnData.scalars);
  } else {
    // this will atomatically set vertex.data to 0 on barney side
    bnSetData(sf, "cell.data", m_bnData.scalars);
  }
  bnSetData(sf, "index", m_bnData.indices);
  bnSetData(sf, "cell.index", m_bnData.elementOffsets);
  bnSetData(sf, "cell.type", m_bnData.cellType);
  bnCommit(sf);
}

BNScalarField UnstructuredField::createBarneyScalarField() const
{
  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = bnScalarFieldCreate(context, slot, "unstructured");
  return sf;
}

box3 UnstructuredField::bounds() const
{
  return m_bounds;
}

bool UnstructuredField::isValid() const
{
  return m_params.vertexPosition && m_params.index && m_params.cellBegin
      && m_params.cellType && (m_params.vertexData || m_params.cellData);
}

// BlockStructuredField //

BlockStructuredField::BlockStructuredField(BarneyGlobalState *s)
    : SpatialField(s), m_params(this)
{}

void BlockStructuredField::commitParameters()
{
  Object::commitParameters();
  m_params.refinementRatio = getParamObject<helium::Array1D>("refinementRatio");
  m_params.blockBounds = getParamObject<helium::Array1D>("block.bounds");
  m_params.blockLevel = getParamObject<helium::Array1D>("block.level");
  m_params.data = getParamObject<helium::Array1D>("data");
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

  if (!m_params.data) {
    reportMessage(ANARI_SEVERITY_WARNING,
        "missing required parameter 'data' on amr spatial field");
    return;
  }

  size_t numBlocks = m_params.blockLevel->totalSize();
  auto *blockBounds = m_params.blockBounds->beginAs<box3i>();
  auto *blockLevels = m_params.blockLevel->beginAs<int>();

  m_generatedBlockOrigins.clear();
  m_generatedBlockDims.clear();
  m_generatedBlockLevels.clear();
  m_generatedBlockOffsets.clear();

  m_bounds.invalidate();

  int maxLevel = 0;
  for (size_t i = 0; i < numBlocks; ++i) {
    const box3i bounds = *(blockBounds + i);
    const int level = *(blockLevels + i);

    math::int3 dims = bounds.upper - bounds.lower + math::int3(1);

    m_generatedBlockOrigins.push_back(bounds.lower);
    m_generatedBlockDims.push_back(dims);
    m_generatedBlockLevels.push_back(level);
    m_generatedBlockOffsets.push_back(dims.x * size_t(dims.y) * dims.z);
    maxLevel = std::max(maxLevel, level);

    box3 worldBounds;
    worldBounds.lower = math::float3(float(bounds.lower.x * (1 << level)),
        float(bounds.lower.y * (1 << level)),
        float(bounds.lower.z * (1 << level)));
    worldBounds.upper = math::float3(float((bounds.upper.x + 1) * (1 << level)),
        float((bounds.upper.y + 1) * (1 << level)),
        float((bounds.upper.z + 1) * (1 << level)));
    m_bounds.insert(worldBounds);
  }

  m_generatedRefinements.resize(maxLevel+1, 2);

  std::exclusive_scan(m_generatedBlockOffsets.begin(),
                      m_generatedBlockOffsets.end(),
                      m_generatedBlockOffsets.begin(),
                      (uint64_t)0);

  //=======================================================
  // get (or create) and populate bn field
  //=======================================================

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = getBarneyScalarField();

  size_t numScalars = m_params.data->size();
  size_t numLevels = m_generatedRefinements.size();

  if (!m_bnData.scalars) {
    m_bnData.scalars =
      bnDataCreate(context, slot, BN_FLOAT32, numScalars, m_params.data->beginAs<float>());
  } else {
    bnDataSet(m_bnData.scalars, numScalars, m_params.data->beginAs<float>());
  }


  if (!m_bnData.blockOrigins) {
    m_bnData.blockOrigins =
      bnDataCreate(context, slot, BN_INT32_VEC3, numBlocks, m_generatedBlockOrigins.data());
  } else {
    bnDataSet(m_bnData.blockOrigins, numBlocks, m_generatedBlockOrigins.data());
  }


  if (!m_bnData.blockDims) {
    m_bnData.blockDims =
      bnDataCreate(context, slot, BN_INT32_VEC3, numBlocks, m_generatedBlockDims.data());
  } else {
    bnDataSet(m_bnData.blockDims, numBlocks, m_generatedBlockDims.data());
  }


  if (!m_bnData.blockLevels) {
    m_bnData.blockLevels =
      bnDataCreate(context, slot, BN_INT32, numBlocks, m_generatedBlockLevels.data());
  } else {
    bnDataSet(m_bnData.blockLevels, numBlocks, m_generatedBlockLevels.data());
  }


  if (!m_bnData.blockOffsets) {
    m_bnData.blockOffsets =
      bnDataCreate(context, slot, BN_UINT64, numBlocks, m_generatedBlockOffsets.data());
  } else {
    bnDataSet(m_bnData.blockOffsets, numBlocks, m_generatedBlockOffsets.data());
  }

  if (!m_bnData.levelRefinements) {
    m_bnData.levelRefinements =
      bnDataCreate(context, slot, BN_INT32, numLevels, m_generatedRefinements.data());
  } else {
    bnDataSet(m_bnData.levelRefinements, numLevels, m_generatedRefinements.data());
  }

  bnSetData(sf, "scalars", m_bnData.scalars);
  bnSetData(sf, "grid.origins", m_bnData.blockOrigins);
  bnSetData(sf, "grid.dims", m_bnData.blockDims);
  bnSetData(sf, "grid.levels", m_bnData.blockLevels);
  bnSetData(sf, "grid.offsets", m_bnData.blockOffsets);
  bnSetData(sf, "level.refinements", m_bnData.levelRefinements);
  bnCommit(sf);
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

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  BNScalarField sf = bnScalarFieldCreate(context, slot, "BlockStructuredAMR");
  return sf;
}

box3 BlockStructuredField::bounds() const
{
  return m_bounds;
}

// PlanetSpatialField //

PlanetSpatialField::PlanetSpatialField(BarneyGlobalState *s)
    : SpatialField(s),
      m_elevationMap(this),
      m_diffuseMap(this),
      m_normalMap(this)
{}

void PlanetSpatialField::commitParameters()
{
  Object::commitParameters();
  m_elevationMap = getParamObject<helium::Array2D>(DEFAULT_ATTR_ELEVATION_MAP);
  m_diffuseMap = getParamObject<helium::Array2D>(DEFAULT_ATTR_DIFFUSE_MAP);
  m_normalMap = getParamObject<helium::Array2D>(DEFAULT_ATTR_NORMAL_MAP);

  m_planetRadius = std::clamp(
      getParam<float>(DEFAULT_ATTR_PLANET_RADIUS, DEFAULT_PLANET_RADIUS),
      0.f,
      1.f);
  m_elevationScale = std::clamp(
      getParam<float>(DEFAULT_ATTR_ELEVATION_SCALE, DEFAULT_ELEVATION_SCALE),
      0.f,
      1.f);

  if (m_sf) {
    bnSet1f(m_sf, DEFAULT_ATTR_PLANET_RADIUS, m_planetRadius);
    bnSet1f(m_sf, DEFAULT_ATTR_ELEVATION_SCALE, m_elevationScale);
  }
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

  m_sf = bnScalarFieldCreate(context, slot, "planet");

  // Set planet parameters
  bnSet1f(m_sf, DEFAULT_ATTR_PLANET_RADIUS, m_planetRadius);
  bnSet1f(m_sf, DEFAULT_ATTR_ELEVATION_SCALE, m_elevationScale);

  // Set texture maps if provided
  if (m_elevationMap) {
    BNTextureData td = bnTextureData2DCreate(context,
        slot,
        BN_FLOAT32,
        m_elevationMap->size().x,
        m_elevationMap->size().y,
        m_elevationMap->data());
    bnSetObject(m_sf, DEFAULT_ATTR_ELEVATION_MAP, td);
    bnRelease(td);
  }

  if (m_diffuseMap) {
    BNTextureData td = bnTextureData2DCreate(context,
        slot,
        BN_FLOAT32_VEC4,
        m_diffuseMap->size().x,
        m_diffuseMap->size().y,
        m_diffuseMap->data());
    bnSetObject(m_sf, DEFAULT_ATTR_DIFFUSE_MAP, td);
    bnRelease(td);
  }

  if (m_normalMap) {
    BNTextureData td = bnTextureData2DCreate(context,
        slot,
        BN_FLOAT32_VEC4,
        m_normalMap->size().x,
        m_normalMap->size().y,
        m_normalMap->data());
    bnSetObject(m_sf, DEFAULT_ATTR_NORMAL_MAP, td);
    bnRelease(td);
  }

  bnCommit(m_sf);
  return m_sf;
}

box3 PlanetSpatialField::bounds() const
{
  const float totalRadius = m_planetRadius + m_elevationScale;
  return box3(-math::float3(totalRadius), math::float3(totalRadius));
}

// CloudSpatialField //

CloudSpatialField::CloudSpatialField(BarneyGlobalState *s)
    : SpatialField(s), m_cloudData(this)
{}

void CloudSpatialField::commitParameters()
{
  Object::commitParameters();

  m_cloudData = getParamObject<helium::Array3D>(DEFAULT_ATTR_CLOUD_DATA);

  m_planetRadius = std::clamp(
      getParam<float>(DEFAULT_ATTR_PLANET_RADIUS, DEFAULT_PLANET_RADIUS),
      0.f,
      1.f);
  m_atmosphereThickness =
      std::clamp(getParam<float>(DEFAULT_ATTR_ATMOSPHERE_THICKNESS,
                     DEFAULT_ATMOSPHERE_THICKNESS),
          0.f,
          1.f);

  // Update parameters if scalar field already exists
  if (m_sf) {
    bnSet1f(m_sf, DEFAULT_ATTR_PLANET_RADIUS, m_planetRadius);
    bnSet1f(m_sf, DEFAULT_ATTR_ATMOSPHERE_THICKNESS, m_atmosphereThickness);
    
    // Set cloud data texture if provided
    if (m_cloudData) {
      BNTextureData td = bnTextureData3DCreate(deviceState()->tether->context,
          deviceState()->slot,
          BN_FLOAT32,
          m_cloudData->size().x,
          m_cloudData->size().y,
          m_cloudData->size().z,
          m_cloudData->data());
      bnSetObject(m_sf, DEFAULT_ATTR_CLOUD_DATA, td);
      bnRelease(td);
    }
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

  m_sf = bnScalarFieldCreate(context, slot, "clouds");

  // Set cloud parameters
  bnSet1f(m_sf, DEFAULT_ATTR_PLANET_RADIUS, m_planetRadius);
  bnSet1f(m_sf, DEFAULT_ATTR_ATMOSPHERE_THICKNESS, m_atmosphereThickness);

  // Set cloud data texture if provided
  if (m_cloudData) {
    BNTextureData td = bnTextureData3DCreate(context,
        slot,
        BN_FLOAT32,
        m_cloudData->size().x,
        m_cloudData->size().y,
        m_cloudData->size().z,
        m_cloudData->data());
    bnSetObject(m_sf, DEFAULT_ATTR_CLOUD_DATA, td);
    bnRelease(td);
  }

  bnCommit(m_sf);
  return m_sf;
}

box3 CloudSpatialField::bounds() const
{
  const float totalRadius = m_planetRadius + m_atmosphereThickness;
  return box3(-math::float3(totalRadius), math::float3(totalRadius));
}

// MagneticSpatialField //

MagneticSpatialField::MagneticSpatialField(BarneyGlobalState *s)
    : SpatialField(s)
{}

void MagneticSpatialField::commitParameters()
{
  Object::commitParameters();

  m_equatorStrength = getParam<float>(DEFAULT_ATTR_EQUATOR_STRENGTH, DEFAULT_EQUATOR_STRENGTH);
  m_poleStrength = getParam<float>(DEFAULT_ATTR_POLE_STRENGTH, DEFAULT_POLE_STRENGTH);
  m_dipoleTilt = getParam<float>(DEFAULT_ATTR_DIPOLE_TILT, DEFAULT_DIPOLE_TILT);

  if (m_sf) {
    bnSet1f(m_sf, DEFAULT_ATTR_EQUATOR_STRENGTH, m_equatorStrength);
    bnSet1f(m_sf, DEFAULT_ATTR_POLE_STRENGTH, m_poleStrength);
    bnSet1f(m_sf, DEFAULT_ATTR_DIPOLE_TILT, m_dipoleTilt);
  }
}

void MagneticSpatialField::finalize()
{
  // Magnetic field is always valid - provides Earth's dipole field computation
}

bool MagneticSpatialField::isValid() const
{
  return true; // Magnetic field provides analytical dipole field
}

BNScalarField MagneticSpatialField::createBarneyScalarField() const
{
  if (!isValid())
    return {};

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  m_sf = bnScalarFieldCreate(context, slot, "magnetic");

  // Set magnetic field parameters
  bnSet1f(m_sf, DEFAULT_ATTR_EQUATOR_STRENGTH, m_equatorStrength);
  bnSet1f(m_sf, DEFAULT_ATTR_POLE_STRENGTH, m_poleStrength);
  bnSet1f(m_sf, DEFAULT_ATTR_DIPOLE_TILT, m_dipoleTilt);

  bnCommit(m_sf);
  return m_sf;
}

box3 MagneticSpatialField::bounds() const
{
  // Earth's magnetic field extends well beyond the planet
  // Use a sphere with radius ~3 Earth radii to capture magnetosphere
  const float magnetosphereRadius = 3.0f;
  return box3(-math::float3(magnetosphereRadius), math::float3(magnetosphereRadius));
}

// AuroraSpatialField //

AuroraSpatialField::AuroraSpatialField(BarneyGlobalState *s)
    : SpatialField(s)
{}

void AuroraSpatialField::commitParameters()
{
  Object::commitParameters();

  m_intensity = getParam<float>(DEFAULT_ATTR_INTENSITY, DEFAULT_INTENSITY);
  m_waveFrequency = getParam<float>(DEFAULT_ATTR_WAVE_FREQUENCY, DEFAULT_WAVE_FREQUENCY);
  m_waveAmplitude = getParam<float>(DEFAULT_ATTR_WAVE_AMPLITUDE, DEFAULT_WAVE_AMPLITUDE);
  m_time = getParam<float>(DEFAULT_ATTR_TIME, DEFAULT_TIME);
  m_altitudeMin = getParam<float>(DEFAULT_ATTR_ALTITUDE_MIN, DEFAULT_ALTITUDE_MIN);
  m_altitudeMax = getParam<float>(DEFAULT_ATTR_ALTITUDE_MAX, DEFAULT_ALTITUDE_MAX);
  m_thickness = getParam<float>(DEFAULT_ATTR_THICKNESS, DEFAULT_THICKNESS);
  m_turbulence = getParam<float>(DEFAULT_ATTR_TURBULENCE, DEFAULT_TURBULENCE);
  m_numCurtains = getParam<float>(DEFAULT_ATTR_NUM_CURTAINS, DEFAULT_NUM_CURTAINS);
  m_magneticLatitude = getParam<float>(DEFAULT_ATTR_MAGNETIC_LATITUDE, DEFAULT_MAGNETIC_LATITUDE);

  if (m_sf) {
    bnSet1f(m_sf, DEFAULT_ATTR_INTENSITY, m_intensity);
    bnSet1f(m_sf, DEFAULT_ATTR_WAVE_FREQUENCY, m_waveFrequency);
    bnSet1f(m_sf, DEFAULT_ATTR_WAVE_AMPLITUDE, m_waveAmplitude);
    bnSet1f(m_sf, DEFAULT_ATTR_TIME, m_time);
    bnSet1f(m_sf, DEFAULT_ATTR_ALTITUDE_MIN, m_altitudeMin);
    bnSet1f(m_sf, DEFAULT_ATTR_ALTITUDE_MAX, m_altitudeMax);
    bnSet1f(m_sf, DEFAULT_ATTR_THICKNESS, m_thickness);
    bnSet1f(m_sf, DEFAULT_ATTR_TURBULENCE, m_turbulence);
    bnSet1f(m_sf, DEFAULT_ATTR_NUM_CURTAINS, m_numCurtains);
    bnSet1f(m_sf, DEFAULT_ATTR_MAGNETIC_LATITUDE, m_magneticLatitude);
  }
}

void AuroraSpatialField::finalize()
{
  // Aurora field is always valid - provides procedural aurora computation
}

bool AuroraSpatialField::isValid() const
{
  return true; // Aurora field provides procedural aurora generation
}

BNScalarField AuroraSpatialField::createBarneyScalarField() const
{
  if (!isValid())
    return {};

  int slot = deviceState()->slot;
  auto context = deviceState()->tether->context;

  m_sf = bnScalarFieldCreate(context, slot, "aurora");

  // Set aurora parameters
  bnSet1f(m_sf, DEFAULT_ATTR_INTENSITY, m_intensity);
  bnSet1f(m_sf, DEFAULT_ATTR_WAVE_FREQUENCY, m_waveFrequency);
  bnSet1f(m_sf, DEFAULT_ATTR_WAVE_AMPLITUDE, m_waveAmplitude);
  bnSet1f(m_sf, DEFAULT_ATTR_TIME, m_time);
  bnSet1f(m_sf, DEFAULT_ATTR_ALTITUDE_MIN, m_altitudeMin);
  bnSet1f(m_sf, DEFAULT_ATTR_ALTITUDE_MAX, m_altitudeMax);
  bnSet1f(m_sf, DEFAULT_ATTR_THICKNESS, m_thickness);
  bnSet1f(m_sf, DEFAULT_ATTR_TURBULENCE, m_turbulence);
  bnSet1f(m_sf, DEFAULT_ATTR_NUM_CURTAINS, m_numCurtains);
  bnSet1f(m_sf, DEFAULT_ATTR_MAGNETIC_LATITUDE, m_magneticLatitude);

  bnCommit(m_sf);
  return m_sf;
}

box3 AuroraSpatialField::bounds() const
{
  // Aurora appears in atmospheric shell
  const float auroraRadius = m_altitudeMax;
  return box3(-math::float3(auroraRadius), math::float3(auroraRadius));
}

} // namespace barney_device

BARNEY_ANARI_TYPEFOR_DEFINITION(barney_device::SpatialField *);
