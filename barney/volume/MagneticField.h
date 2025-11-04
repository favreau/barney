// ======================================================================== //
// Copyright 2023-2025 Ingo Wald                                            //
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
  /*! Scalar field representing Earth's magnetic field as a dipole model.

      The MagneticField computes magnetic field strength based on a
      tilted dipole model of Earth's magnetic field. The field strength
      varies from equator to poles and includes the dipole tilt angle.

      Supported settable fields:

      - "equatorStrength" (float) : Magnetic field strength at equator in nT (default: 31000)
      - "poleStrength" (float) : Magnetic field strength at poles in nT (default: 62000)
      - "dipoleTilt" (float) : Magnetic dipole tilt angle in degrees (default: 11.5)
  */
  struct MagneticField : public ScalarField
  {
    /*! device data for this class */
    struct DD : public ScalarField::DD {
      float equatorStrength;
      float poleStrength;
      float dipoleTilt;
    };

    /*! construct a new magnetic scalar field */
    MagneticField(Context *context,
                  const DevGroup::SP &devices);
    virtual ~MagneticField() = default;

    // ------------------------------------------------------------------
    /*! @{ parameter set/commit interface */
    bool set1f(const std::string &member, const float &value) override;
    bool setObject(const std::string &member, const Object::SP &value) override;
    void commit() override;
    /*! @} */
    // ------------------------------------------------------------------
    
    DD getDD(Device *device);
    VolumeAccel::SP createAccel(Volume *volume) override;
    MCGrid::SP buildMCs() override;

    struct PLD {
      rtc::ComputeKernel3D *computeMCs = 0;
    };
    PLD *getPLD(Device *device);
    std::vector<PLD> perLogical;
    
    // Magnetic field parameters
    float equatorStrength = 31000.0f;  // nT
    float poleStrength = 62000.0f;     // nT
    float dipoleTilt = 11.5f;          // degrees
  };

  /*! sampler object for MagneticField, handling dipole field computation */
  struct MagneticSampler : public ScalarFieldSampler {
    MagneticSampler(MagneticField *const sf)
      : sf(sf)
    {}
    
    struct DD {
#if RTC_DEVICE_CODE
      inline __rtc_device float sample(const vec3f P, bool dbg=false) const;
#endif
      
      float equatorStrength;
      float poleStrength;
      float dipoleTilt;
    };

    void build() override {}
    
    DD getDD(Device *device);
    MagneticField *const sf;
  };
  
#if RTC_DEVICE_CODE
  inline __rtc_device
  float MagneticSampler::DD::sample(const vec3f P, bool dbg) const
  {
    // Convert position to spherical coordinates
    const float r = length(P);
    if (r < 1e-6f) return 0.0f; // Avoid division by zero at origin
    
    const vec3f dir = P / r;
    
    // Compute magnetic colatitude (angle from north pole)
    // Apply dipole tilt by rotating the coordinate system
    const float tiltRad = dipoleTilt * M_PI / 180.0f;
    const float cosTheta = dir.y * cosf(tiltRad) + dir.z * sinf(tiltRad);
    const float theta = acosf(clamp(cosTheta, -1.0f, 1.0f));
    
    // Dipole field strength formula: B(r,θ) = B₀ * (1/r³) * √(1 + 3cos²θ)
    // Where B₀ is the field strength at the magnetic equator at r=1
    const float cosTheta2 = cosTheta * cosTheta;
    const float fieldFactor = sqrtf(1.0f + 3.0f * cosTheta2);
    
    // Interpolate between equator and pole strength based on magnetic latitude
    const float magneticLatitude = fabsf(cosTheta);
    const float baseStrength = equatorStrength * (1.0f - magneticLatitude) + 
                              poleStrength * magneticLatitude;
    
    // Apply 1/r³ distance falloff for dipole field
    const float distanceFactor = 1.0f / (r * r * r);
    
    // Compute final field strength
    const float fieldStrength = baseStrength * fieldFactor * distanceFactor;
    
    // Normalize to 0-1 range for volume rendering
    // Earth's field ranges from ~25,000 to ~65,000 nT
    const float normalizedField = clamp(fieldStrength / 100000.0f, 0.0f, 1.0f);
    
    return normalizedField;
  }
#endif
}
