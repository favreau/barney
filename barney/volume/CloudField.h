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
  /*! Scalar field representing clouds as a spherical volume with
      3D texture-based density distribution.

      The CloudField uses spherical coordinates to sample from a 3D
      texture array where the z-coordinate represents height information.

      Supported settable fields:

      - "cloudData" (BNTexture3D) : 3D float texture array containing cloud density data
      - "planetRadius" (float) : Radius of the base sphere (default: 0.9 for unit volume)
      - "atmosphereThickness" (float) : Maximum height above sphere surface (default: 0.01)
  */
  struct CloudField : public ScalarField
  {
    /*! device data for this class */
    struct DD : public ScalarField::DD {
      float planetRadius;
      float atmosphereThickness;
    };

    /*! construct a new cloud scalar field */
    CloudField(Context *context,
               const DevGroup::SP &devices);
    virtual ~CloudField() = default;

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

    // Cloud texture
    TextureData::SP cloudTextureData;
    Texture::SP cloudData;

    struct CLD {
      rtc::ComputeKernel3D *computeMCs = 0;
    };
    CLD *getCLD(Device *device);
    std::vector<CLD> perLogical;
    
    // Cloud parameters
    float planetRadius = 0.9f;
    float atmosphereThickness = 0.01f;
  };

  /*! sampler object for CloudField, handling spherical coordinate sampling */
  struct CloudSampler : public ScalarFieldSampler {
    CloudSampler(CloudField *const sf)
      : sf(sf)
    {}
    
    struct DD {
#if RTC_DEVICE_CODE
      inline __rtc_device float sample(const vec3f P, bool dbg=false) const;
#endif
      
      rtc::TextureObject cloudDataTex{nullptr};
      vec3i textureDims{0,0,0}; // Add texture dimensions for proper coordinate calculation

      float planetRadius;
      float atmosphereThickness;
    };

    void build() override {}
    
    DD getDD(Device *device);
    CloudField *const sf;
  };
  
#if RTC_DEVICE_CODE
  inline __rtc_device
  float CloudSampler::DD::sample(const vec3f P, bool dbg) const
  {
    float dist = length(P);
    
    // Convert to spherical coordinates
    const vec3f dir = normalize(P);
    const float theta = acosf(clamp(dir.y, -1.0f, 1.0f));
    const float phi = atan2f(dir.z, dir.x) + M_PI;
    const vec2f uv(phi / (2.0f * M_PI), theta / M_PI);
    
    // Calculate height above sphere surface
    const float height = max(0.0f, dist - planetRadius);
    const float normalizedHeight = clamp(height / atmosphereThickness, 0.0f, 1.0f);
    
    // If we're inside the sphere or above max height, no cloud density
    if (dist < planetRadius || normalizedHeight >= 1.0f)
      return 0.0f;
    
    // For 3D textures with non-normalized coordinates, convert to texture space
    // and add 0.5 offset for proper texel center sampling (like StructuredData)
    if (textureDims.x > 0 && textureDims.y > 0 && textureDims.z > 0) {
      // Convert normalized coordinates to texture coordinates and add texel center offset
      float texX = uv.x * float(textureDims.x - 1) + 0.5f;
      float texY = uv.y * float(textureDims.y - 1) + 0.5f;
      float texZ = normalizedHeight * float(textureDims.z - 1) + 0.5f;
      const float density = rtc::tex3D<float>(cloudDataTex, texX, texY, texZ);
      return density;
    }
    return 0.0f; // No valid texture dimensions
  }
#endif

} // namespace BARNEY_NS 