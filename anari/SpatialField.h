// Copyright 2023 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari
#include "helium/array/Array1D.h"
#include "helium/array/Array3D.h"
#include "helium/array/ObjectArray.h"
// ours
#include "Object.h"

namespace barney_device {

  struct SpatialField : public Object
  {
    SpatialField(BarneyGlobalState *s);
    ~SpatialField() override;

    static SpatialField *createInstance(std::string_view subtype,
                                        BarneyGlobalState *s);

    void markFinalized() override;

    virtual BNScalarField createBarneyScalarField() const = 0;

    void cleanup()
    {
      if (m_bnField) {
        bnRelease(m_bnField);
        m_bnField = nullptr;
      }
    }

    BNScalarField getBarneyScalarField()
    {
      if (!isValid())
        return {};
      if (!m_bnField)
        m_bnField = createBarneyScalarField();
      return m_bnField;
    }

    virtual box3 bounds() const = 0;

    BNScalarField m_bnField = 0;
  };

  // Subtypes ///////////////////////////////////////////////////////////////////

  struct UnstructuredField : public SpatialField
  {
    UnstructuredField(BarneyGlobalState *s);

    void commitParameters() override;
    void finalize() override;

    BNScalarField createBarneyScalarField() const override;

    box3 bounds() const override;

  private:
    struct Parameters
    {
      helium::IntrusivePtr<helium::Array1D> vertexPosition;
      helium::IntrusivePtr<helium::Array1D> vertexData;
      helium::IntrusivePtr<helium::Array1D> cellData;
      helium::IntrusivePtr<helium::Array1D> index;
      helium::IntrusivePtr<helium::Array1D> cellType;
      helium::IntrusivePtr<helium::Array1D> cellBegin;
    } m_params;

    box3 m_bounds;
  };

  struct BlockStructuredField : public SpatialField
  {
    BlockStructuredField(BarneyGlobalState *s);
    void commitParameters() override;
    void finalize() override;

    BNScalarField createBarneyScalarField() const override;

    box3 bounds() const override;

    struct Parameters
    {
      helium::IntrusivePtr<helium::Array1D> cellWidth;
      helium::IntrusivePtr<helium::Array1D> blockBounds;
      helium::IntrusivePtr<helium::Array1D> blockLevel;
      helium::IntrusivePtr<helium::ObjectArray> blockData;
    } m_params;

    std::vector<int> m_generatedBlockBounds;
    std::vector<int> m_generatedBlockLevels;
    std::vector<int> m_generatedBlockOffsets;
    std::vector<float> m_generatedBlockScalars;

    box3 m_bounds;
  };

  struct StructuredRegularField : public SpatialField
  {
    StructuredRegularField(BarneyGlobalState *s);
    void commitParameters() override;
    void finalize() override;

    BNScalarField createBarneyScalarField() const override;

    box3 bounds() const override;
    bool isValid() const override;

    math::uint3 m_dims{0u};
    math::float3 m_origin;
    math::float3 m_spacing;
    math::float3 m_coordUpperBound;

    std::vector<float> m_generatedCellWidths;
    std::vector<int> m_generatedBlockBounds;
    std::vector<int> m_generatedBlockLevels;
    std::vector<int> m_generatedBlockOffsets;
    std::vector<float> m_generatedBlockScalars;

    helium::IntrusivePtr<helium::Array3D> m_data;
  };

  struct PlanetSpatialField : public SpatialField
  {
    PlanetSpatialField(BarneyGlobalState *s);
    void commitParameters() override;
    void finalize() override;

    BNScalarField createBarneyScalarField() const override;

    box3 bounds() const override;
    bool isValid() const override;

    mutable BNScalarField m_sf{nullptr};

    static constexpr float DEFAULT_PLANET_RADIUS = 0.9f;
    static constexpr float DEFAULT_ELEVATION_SCALE = 0.1f;
    
    static constexpr const char* VOLUME_SUBTYPE = "planet";
    static constexpr const char* DEFAULT_ATTR_PLANET_RADIUS = "planetRadius";
    static constexpr const char* DEFAULT_ATTR_ELEVATION_SCALE = "elevationScale";
    static constexpr const char* DEFAULT_ATTR_ELEVATION_MAP = "elevationMap";
    static constexpr const char* DEFAULT_ATTR_DIFFUSE_MAP = "diffuseMap";
    static constexpr const char* DEFAULT_ATTR_NORMAL_MAP = "normalMap";

    float m_planetRadius{DEFAULT_PLANET_RADIUS};
    float m_elevationScale{DEFAULT_ELEVATION_SCALE};

    helium::IntrusivePtr<Array2D> m_elevationMap;
    helium::IntrusivePtr<Array2D> m_diffuseMap;
    helium::IntrusivePtr<Array2D> m_normalMap;
  };

  struct CloudSpatialField : public SpatialField
  {
    CloudSpatialField(BarneyGlobalState *s);
    void commitParameters() override;
    void finalize() override;

    BNScalarField createBarneyScalarField() const override;

    box3 bounds() const override;
    bool isValid() const override;

    mutable BNScalarField m_sf{nullptr};

    static constexpr const char* VOLUME_SUBTYPE = "cloud";
    static constexpr const char* DEFAULT_ATTR_CLOUD_DATA = "cloudData";
    static constexpr const char* DEFAULT_ATTR_PLANET_RADIUS = "planetRadius";
    static constexpr const char* DEFAULT_ATTR_ATMOSPHERE_THICKNESS = "atmosphereThickness";

    static constexpr float DEFAULT_PLANET_RADIUS = 0.9f;
    static constexpr float DEFAULT_ATMOSPHERE_THICKNESS = 0.01f;

    float m_planetRadius{DEFAULT_PLANET_RADIUS};
    float m_atmosphereThickness{DEFAULT_ATMOSPHERE_THICKNESS};

    helium::IntrusivePtr<Array3D> m_cloudData;
  };

} // namespace barney_device

BARNEY_ANARI_TYPEFOR_SPECIALIZATION(barney_device::SpatialField *,
                                    ANARI_SPATIAL_FIELD);
