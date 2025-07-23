// ======================================================================== //
// Copyright 2023-2024 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "barney/ModelSlot.h"
#include "barney/common/Texture.h"
#include "barney/volume/MCAccelerator.h"

namespace BARNEY_NS {

  struct ModelSlot;

  // ==================================================================
  /*! Scalar field representing a planet as a unit sphere with
      layered structure and texture maps.

      The Planet volume has multiple layers:
      - Core (0.0 - 0.35 radius)
      - Mantle (0.35 - 0.9 radius) 
      - Crust/Surface (0.9 - 1.0 radius + elevation)
      - Atmosphere (1.0 - 1.2 radius)

      Supported settable fields:

      - "elevationMap" (BNTexture2D) : Elevation/height map in equirectangular projection
      - "diffuseMap" (BNTexture2D) : Surface color/albedo map in equirectangular projection
      - "normalMap" (BNTexture2D) : Surface normal map in equirectangular projection
      - "planetRadius" (float) : Radius of the base sphere (default: 0.5 for unit volume)
      - "elevationScale" (float) : Scale factor for elevation values (default: 0.1)
  */
  struct PlanetField : public ScalarField
  {
    /*! device data for this class */
    struct DD : public ScalarField::DD {
      float planetRadius;
      float elevationScale;
    };

    /*! construct a new planet scalar field */
    PlanetField(Context *context,
               const DevGroup::SP &devices);
    virtual ~PlanetField() = default;

    // ------------------------------------------------------------------
    /*! @{ parameter set/commit interface */
    bool set1f(const std::string &member, const float &value) override;
    bool setObject(const std::string &member, const Object::SP &value) override;
    void commit() override;
    /*! @} */
    // ------------------------------------------------------------------
    
    DD getDD(Device *device);
    VolumeAccel::SP createAccel(Volume *volume) override;
    void buildMCs(MCGrid &macroCells) override;

    // Planet textures
    TextureData::SP elevationData;
    TextureData::SP diffuseData;
    TextureData::SP normalData;
    
    Texture::SP elevationMap;
    Texture::SP diffuseMap;
    Texture::SP normalMap;

    struct PLD {
      rtc::ComputeKernel3D *computeMCs = 0;
    };
    PLD *getPLD(Device *device);
    std::vector<PLD> perLogical;
    
    // Planet parameters
    float planetRadius = 0.5f;
    float elevationScale = 0.1f;
  };

  /*! sampler object for PlanetField, handling spherical coordinate sampling */
  struct PlanetSampler : public ScalarFieldSampler {
    PlanetSampler(PlanetField *const sf)
      : sf(sf)
    {}
    
    struct DD {
#if RTC_DEVICE_CODE
      inline __rtc_device float sample(const vec3f P, bool dbg=false) const;
#endif
      
      rtc::TextureObject elevationTex{nullptr};
      rtc::TextureObject diffuseTex{nullptr}; 
      rtc::TextureObject normalTex{nullptr};
      float elevationScale;
      float planetRadius;
    };

    void build() override {}
    
    DD getDD(Device *device);
    PlanetField *const sf;
  };
  
#if RTC_DEVICE_CODE
  inline __rtc_device
  float PlanetSampler::DD::sample(const vec3f P, bool dbg) const
  {
    float dist = length(P);
    
    // Convert to spherical coordinates for elevation lookup
    const vec3f dir = normalize(P);
    const float theta = acosf(clamp(dir.y, -1.0f, 1.0f));
    const float phi = atan2f(dir.z, dir.x) + M_PI;
    const vec2f uv(phi / (2.0f * M_PI), theta / M_PI);
    
    // Get surface elevation
    const float elevation = rtc::tex2D<float>(elevationTex, uv.x, uv.y);
    
    const float innerCoreRadius = 0.127f;
    const float outterCoreRadius = innerCoreRadius + 0.220f;
    const float mantleRadius = outterCoreRadius + 0.285f;

    const float surfaceElevation = planetRadius + elevationScale * elevation;
    if (dist < innerCoreRadius) {
      return 0.02f;
    }
    else if (dist < outterCoreRadius) {
      return 0.04f;
    }
    else if (dist < mantleRadius) {
      return 0.06f;
    }
    else if (dist < planetRadius) {
      // Between core and surface
      return 0.8f;
    }
    else if (dist >= planetRadius && dist < surfaceElevation) {
      // Surface
      return 0.1f + 0.8f * elevation;
    }
    
    return 0.0f;
  }
#endif
} 