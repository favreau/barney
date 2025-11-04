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
  /*! Scalar field representing Aurora Borealis/Australis in Earth's atmosphere.

      The AuroraField generates realistic Aurora (Northern/Southern Lights) effects
      by simulating the interaction between solar wind particles and Earth's 
      magnetic field. Aurora appears in oval bands around magnetic poles with
      dynamic curtain-like structures.

      Supported settable fields:

      - "intensity" (float) : Overall aurora brightness (default: 10.0)
      - "waveFrequency" (float) : Curtain wave frequency (default: 25.0)
      - "waveAmplitude" (float) : Curtain wave amplitude (default: 8.0)
      - "time" (float) : Animation time parameter (default: 0.0)
      - "altitudeMin" (float) : Minimum altitude radius in units (default: 1.1)
      - "altitudeMax" (float) : Maximum altitude radius in units (default: 1.3)
      - "thickness" (float) : Aurora curtain thickness (default: 0.05)
      - "turbulence" (float) : Turbulence/noise intensity (default: 0.1)
      - "numCurtains" (float) : Number of aurora curtains (default: 8.0)
      - "magneticLatitude" (float) : Magnetic latitude offset in degrees (default: 0.0)
  */
  struct AuroraField : public ScalarField
  {
    /*! device data for this class */
    struct DD : public ScalarField::DD {
      float intensity;
      float waveFrequency;
      float waveAmplitude;
      float time;
      float altitudeMin;
      float altitudeMax;
      float thickness;
      float turbulence;
      float numCurtains;
      float magneticLatitude;
    };

    /*! construct a new aurora scalar field */
    AuroraField(Context *context,
                const DevGroup::SP &devices);
    virtual ~AuroraField() = default;

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
    
    // Aurora parameters (scaled to match CloudField conventions)
    float intensity = 1.0f;        // Reduced for proper visibility
    float waveFrequency = 25.0f;
    float waveAmplitude = 0.005f;  // Very small for unit volume
    float time = 0.0f;
    float altitudeMin = 0.91f;     // Just above cloud layer (0.9+0.01)
    float altitudeMax = 0.94f;     // Thin atmospheric shell
    float thickness = 0.002f;      // Very thin curtains for unit volume
    float turbulence = 0.1f;
    float numCurtains = 8.0f;
    float magneticLatitude = 0.0f;
  };

  /*! sampler object for AuroraField, handling aurora field computation */
  struct AuroraSampler : public ScalarFieldSampler {
    AuroraSampler(AuroraField *const sf)
      : sf(sf)
    {}
    
    struct DD {
#if RTC_DEVICE_CODE
      inline __rtc_device float sample(const vec3f P, bool dbg=false) const;
#endif
      
      float intensity;
      float waveFrequency;
      float waveAmplitude;
      float time;
      float altitudeMin;
      float altitudeMax;
      float thickness;
      float turbulence;
      float numCurtains;
      float magneticLatitude;
    };

    void build() override {}
    
    DD getDD(Device *device);
    AuroraField *const sf;
  };
  
#if RTC_DEVICE_CODE
  // ========================================================================
  // DEVICE CODE - Aurora Field Sampling
  // ========================================================================
  
  inline __rtc_device
  float aurora_fract(float x)
  {
    return x - floorf(x);
  }
  
  inline __rtc_device
  float hash31(const vec3f &p)
  {
    float h = dot(p, vec3f(127.1f, 311.7f, 74.7f));
    return aurora_fract(sinf(h) * 43758.5453123f);
  }
  
  inline __rtc_device
  float noise3d(const vec3f &p)
  {
    vec3f i = vec3f(floorf(p.x), floorf(p.y), floorf(p.z));
    vec3f f = vec3f(aurora_fract(p.x), aurora_fract(p.y), aurora_fract(p.z));
    
    // Smooth interpolation
    vec3f u = f * f * (vec3f(3.0f) - f * 2.0f);
    
    // Hash-based random values at corners
    float n000 = hash31(i + vec3f(0.0f, 0.0f, 0.0f));
    float n001 = hash31(i + vec3f(0.0f, 0.0f, 1.0f));
    float n010 = hash31(i + vec3f(0.0f, 1.0f, 0.0f));
    float n011 = hash31(i + vec3f(0.0f, 1.0f, 1.0f));
    float n100 = hash31(i + vec3f(1.0f, 0.0f, 0.0f));
    float n101 = hash31(i + vec3f(1.0f, 0.0f, 1.0f));
    float n110 = hash31(i + vec3f(1.0f, 1.0f, 0.0f));
    float n111 = hash31(i + vec3f(1.0f, 1.0f, 1.0f));
    
    // Trilinear interpolation
    float x00 = lerp_r(n000, n100, u.x);
    float x01 = lerp_r(n001, n101, u.x);
    float x10 = lerp_r(n010, n110, u.x);
    float x11 = lerp_r(n011, n111, u.x);
    
    float y0 = lerp_r(x00, x10, u.y);
    float y1 = lerp_r(x01, x11, u.y);
    
    return lerp_r(y0, y1, u.z) * 2.0f - 1.0f;
  }
  
  inline __rtc_device
  float calculate_turbulence(const vec3f &pos, float time, float turbulence)
  {
    float turb = 0.0f;
    float freq = 4.0f;
    float amp = 1.0f;
    
    for (int i = 0; i < 3; ++i) {
      vec3f scaled_pos = pos * freq;
      float time_scale = 0.1f * (1.0f + float(i) * 0.3f);
      vec3f time_offset = vec3f(time * time_scale, 
                                time * time_scale * 0.8f,
                                time * time_scale * 1.2f);
      turb += noise3d(scaled_pos + time_offset) * amp;
      freq *= 2.0f;
      amp *= 0.5f;
    }
    
    return (turb + 1.0f) * 0.5f;
  }
  
  inline __rtc_device
  float calculate_spherical_curtain(const vec3f &dir, float longitude, float mag_lat, 
                                   float radius, float offset, float time,
                                   float waveFrequency, float waveAmplitude,
                                   float thickness, float altitudeMin, float altitudeMax)
  {
    
    // Each curtain centered at specific longitude
    float curtain_longitude = offset * TWO_PI + time * 0.05f;
    
    // Calculate angular distance
    float lon_distance = longitude - curtain_longitude;
    if (lon_distance > M_PI) lon_distance -= TWO_PI;
    if (lon_distance < -M_PI) lon_distance += TWO_PI;
    
    // Wave-like variations
    float wave_phase1 = mag_lat * waveFrequency + time * 0.5f + offset * M_PI;
    float wave_phase2 = mag_lat * waveFrequency * 1.3f - time * 0.3f;
    float wave_offset = sinf(wave_phase1) * waveAmplitude +
                       sinf(wave_phase2) * waveAmplitude * 0.5f;
    
    lon_distance += wave_offset;
    
    // Thickness with pulsing
    float thickness_variation = 1.0f + sinf(time * 0.3f + offset * TWO_PI) * 0.2f;
    float effective_thickness = thickness * thickness_variation;
    float curtain_strength = expf(-lon_distance * lon_distance / 
                                  (effective_thickness * effective_thickness));
    
    // Ripples along field lines
    float ripple1 = sinf(mag_lat * 20.0f + time * 2.0f) * 0.1f + 0.9f;
    float ripple2 = sinf(mag_lat * 10.0f - time * 1.5f + offset * M_PI) * 0.05f + 0.95f;
    curtain_strength *= ripple1 * ripple2;
    
    // Radial variation
    float radial_phase = (radius - altitudeMin) / (altitudeMax - altitudeMin) * M_PI * 3.0f + time * 0.8f;
    float radial_mod = sinf(radial_phase) * 0.3f + 0.7f;
    curtain_strength *= radial_mod;
    
    // Pulsing
    float pulse = sinf(time * 0.5f + offset * TWO_PI) * 0.2f + 0.8f;
    curtain_strength *= pulse;
    
    // Latitudinal fading
    float lat_fade = 1.0f - fabsf(mag_lat) / (M_PI * 0.5f) * 0.3f;
    curtain_strength *= lat_fade;
    
    return curtain_strength;
  }
  
  inline __rtc_device
  float AuroraSampler::DD::sample(const vec3f P, bool dbg) const
  {
    
    // Calculate distance from origin (Earth center)
    const float dist_from_center = length(P);
    
    // Early exit if outside aurora altitude range
    if (dist_from_center < altitudeMin || dist_from_center > altitudeMax)
      return 0.0f;
    
    // Radial altitude falloff
    float altitude_center = (altitudeMin + altitudeMax) * 0.5f;
    float altitude_range = altitudeMax - altitudeMin;
    float altitude_factor = 1.0f - fabsf(dist_from_center - altitude_center) / (altitude_range * 0.5f);
    altitude_factor = clamp(altitude_factor, 0.0f, 1.0f);
    altitude_factor = altitude_factor * altitude_factor * (3.0f - 2.0f * altitude_factor); // smoothstep
    
    // Convert to spherical coordinates
    vec3f dir = normalize(P);
    
    // Magnetic axis with tilt
    float magnetic_tilt_rad = magneticLatitude * M_PI / 180.0f;
    vec3f magnetic_north = vec3f(sinf(magnetic_tilt_rad), cosf(magnetic_tilt_rad), 0.0f);
    
    // Magnetic latitude
    float mag_lat = asinf(clamp(dot(dir, magnetic_north), -1.0f, 1.0f));
    float abs_mag_lat = fabsf(mag_lat);
    
    // Aurora bands around poles (60-75 degrees)
    float aurora_lat_center = 67.0f * M_PI / 180.0f;
    float aurora_lat_width = 10.0f * M_PI / 180.0f;
    
    float lat_distance = fabsf(abs_mag_lat - aurora_lat_center);
    float latitude_factor = expf(-lat_distance * lat_distance / (aurora_lat_width * aurora_lat_width));
    
    if (latitude_factor < 0.01f)
      return 0.0f;
    
    // Calculate longitude
    float longitude = atan2f(P.x, P.z);
    
    // Generate multiple aurora curtains
    float curtain_value = 0.0f;
    for (float i = 0.0f; i < numCurtains; i += 1.0f) {
      float curtain_offset = i / numCurtains;
      curtain_value += calculate_spherical_curtain(dir, longitude, mag_lat, dist_from_center,
                                                   curtain_offset, time, waveFrequency, 
                                                   waveAmplitude, thickness, altitudeMin, altitudeMax);
    }
    curtain_value /= numCurtains;
    
    // Combine factors
    float aurora_field = curtain_value * altitude_factor * latitude_factor;
    
    // Add turbulence
    float turb = calculate_turbulence(P, time, turbulence);
    aurora_field *= (1.0f - turbulence * 0.5f + turbulence * turb);
    
    // Global pulsing
    float global_pulse = 1.0f + sinf(time * 0.2f) * 0.15f + sinf(time * 0.37f) * 0.1f;
    aurora_field *= global_pulse;
    
    // Apply intensity
    aurora_field *= intensity;
    
    return clamp(aurora_field, 0.0f, 1.0f);
  }
#endif
}

