// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "barney/geometry/Geometry.h"
#include "barney/geometry/SDFPrimitive.h"

namespace BARNEY_NS {

  struct SDFGeometries : public Geometry {
    typedef std::shared_ptr<SDFGeometries> SP;

    struct DD : public Geometry::DD {
      const SDFPrimitive *geometries{nullptr};
      const uint64_t      *neighbours{nullptr};
      uint32_t numGeometries{0};
      float    epsilon{1e-5f};
      uint32_t nbMarchIterations{16};
      float    blendFactor{1.f};
      float    blendLerpFactor{0.5f};
      float    omega{1.f};
      float    distanceFromCamera{100.f};
      float    noiseFactor{0.f};
      float    blendDistanceFromCamera{1e6f};
    };

    SDFGeometries(Context *context, DevGroup::SP devices);

    std::string toString() const override { return "SDFGeometries{}"; }

    void commit() override;

    bool setData(const std::string &member,
                 const barney_api::Data::SP &value) override;
    bool set1f(const std::string &member, const float &value) override;
    bool set1i(const std::string &member, const int &value) override;

    PODData::SP sdf;
    PODData::SP neighbour;

    float    epsilon{1e-5f};
    uint32_t nbMarchIterations{16};
    float    blendFactor{1.f};
    float    blendLerpFactor{0.5f};
    float    omega{1.f};
    float    distanceFromCamera{100.f};
    float    noiseFactor{0.f};
    float    blendDistanceFromCamera{1e6f};
  };

} // namespace BARNEY_NS
