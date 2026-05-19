// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "barney/common/barney-common.h"
#include <cstddef>
#include <cstdint>

namespace BARNEY_NS {

  enum class SDFType : uint8_t {
    SPHERE = 0,
    PILL = 1,
    CONE_PILL = 2,
    CONE_PILL_SIGMOID = 3,
    CONE = 4,
    TORUS = 5,
    CUT_SPHERE = 6,
    VESICA = 7,
    ELLIPSOID = 8
  };

  // Byte layout shared with VisRTX / ANARI UINT8 arrays (72 bytes per primitive).
  struct SDFPrimitive {
    uint64_t userData{0};
    vec3f    userParams{0.f};
    vec3f    p0{0.f};
    vec3f    p1{0.f};
    float    r0{-1.f};
    float    r1{-1.f};
    uint32_t _pad{0};
    uint64_t neighboursIndex{0};
    uint8_t  numNeighbours{0};
    uint8_t  type{0};
    uint8_t  _pad2[6]{};
  };

  static_assert(sizeof(SDFPrimitive) == 72,
                "SDFPrimitive must be 72 bytes for ANARI/VisRTX compatibility");
  static_assert(offsetof(SDFPrimitive, neighboursIndex) == 56,
                "SDFPrimitive::neighboursIndex offset mismatch");

} // namespace BARNEY_NS
