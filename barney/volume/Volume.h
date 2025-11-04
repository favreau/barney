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

#include "barney/common/barney-common.h"
#include "barney/Object.h"
#include "barney/volume/TransferFunction.h"
#include "barney/volume/ScalarField.h"
#include <array>

#define VOLUME_GI

namespace BARNEY_NS {

  using Random = Random2;
  struct Volume;
  struct ScalarField;
  struct MCGrid;
  
  typedef std::array<int,4> TetIndices;
  typedef std::array<int,5> PyrIndices;
  typedef std::array<int,6> WedIndices;
  typedef std::array<int,8> HexIndices;

  struct VolumeAccel {
    
    typedef std::shared_ptr<VolumeAccel> SP;

    VolumeAccel(Volume *volume);

    virtual void build(bool full_rebuild) = 0;

    const TransferFunction *getXF() const;
    
    Volume      *const volume = 0;
    const DevGroup::SP devices;
  };





  struct VolumeAccel;
  
  /*! a *volume* is a scalar field with a transfer function applied to
      it; it's main job is to create something that can intersect a
      ray with that scalars-plus-transferfct thingy, for which it will
      use some kind of volume accelerator that implements the
      scalar-field type specific stuff (eg, traverse a bvh over
      elements, or look up a 3d texture, etc) */
  struct Volume : public barney_api::Volume
  {
    template<typename SFSampler>
    struct DD {
      inline __rtc_device
      vec4f sampleAndMap(vec3f point, bool dbg=false) const
      {
        float f = sfSampler.sample(point,dbg);
        if (isnan(f)) return vec4f(0.f);
        vec4f mapped = xf.map(f,dbg);
        return mapped;
      }
      
      ScalarField::DD        sfCommon;
      typename SFSampler::DD sfSampler;
      TransferFunction::DD   xf;
      int                    userID;
    };
    
    template<typename SFSampler>
    DD<SFSampler> getDD(Device *device, std::shared_ptr<SFSampler> sampler)
    {
      DD<SFSampler> dd;
      dd.sfCommon = sf->getDD(device);
      dd.sfSampler = sampler->getDD(device);
      dd.xf = xf.getDD(device);
      dd.userID = userID;
      return dd;
    }

    typedef std::shared_ptr<Volume> SP;
    
    Volume(ScalarField::SP sf);

    /*! pretty-printer for printf-debugging */
    std::string toString() const override
    { return "Volume{}"; }

    static SP create(ScalarField::SP sf)
    {
      return std::make_shared<Volume>(sf);
    }
    
    /*! (re-)build the accel structure for this volume, probably after
        changes to transfer functoin (or later, scalar field) */
    virtual void build(bool full_rebuild);
    
    void setXF(const range1f &domain,
               const bn_float4 *values,
               int numValues,
               float baseDensity) override;
    bool set1i(const std::string &member,
               const int   &value) override;
               
    ScalarField::SP  sf;
    VolumeAccel::SP  accel;
    TransferFunction xf;
    DevGroup::SP const devices;
    int userID = 0;
    
    struct PLD {
      std::vector<rtc::Group *> generatedGroups;
      std::vector<rtc::Geom *>  generatedGeoms;
    };
    PLD *getPLD(Device *device);
    std::vector<PLD> perLogical;
  };


  inline __rtc_device float fastLog(float f)
  {
    f = (f-1.f)/(f+1.f);
    float f2 = f*f;
    float s = f;
    f *= f2;
    s += (1.f/3.f)*f;
    f *= f2;
    s += (1.f/5.f)*f;
    return s+s;
  }
  
  /*! helper class that performs woodcock sampling over a given
      parameter range, for a given sample'able volume type */
  struct Woodcock {
    template<typename VolumeDD>
    static inline __rtc_device
    bool sampleRange(vec4f &sample,
                     const VolumeDD &sfSampler,
                     vec3f org, vec3f dir,
                     range1f &tRange,
                     float majorant,
                     Random &rand,
                     bool dbg=false) 
    {
      float t = tRange.lower;
      while (true) {
        float r = rand();
        float dt = - fastLog(1.f-r)/majorant;
        // float dt = - logf(1.f-r)/majorant;
        t += dt;
        if (t >= tRange.upper)
          return false;

        vec3f P = org+t*dir;
        sample = sfSampler.sampleAndMap(P,dbg);
        // if (dbg) printf("sample at t %f, P= %f %f %f -> %f %f %f : %f\n",
        //                 t,
        //                 P.x,P.y,P.z,
        //                 sample.x,
        //                 sample.y,
        //                 sample.z,
        //                 sample.w);
        if (sample.w >= rand()*majorant) {
          tRange.upper = t;
          return true;
        }
      }
    }
  };

  inline VolumeAccel::VolumeAccel(Volume *volume)
    : volume(volume),
      devices(volume->devices)
  {
    assert(volume);
    assert(volume->sf);
  }

#ifdef VOLUME_GI
    template <typename VolumeDD>
    static inline __rtc_device
        vec3f
        computeVolumeGradient(const VolumeDD &sfSampler, vec3f P, float h = 0.01f)
    {
      const box3f &bounds = sfSampler.sfCommon.worldBounds;
      
      vec3f P_x_plus = P + vec3f(h, 0, 0);
      vec3f P_x_minus = P - vec3f(h, 0, 0);
      vec3f P_y_plus = P + vec3f(0, h, 0);
      vec3f P_y_minus = P - vec3f(0, h, 0);
      vec3f P_z_plus = P + vec3f(0, 0, h);
      vec3f P_z_minus = P - vec3f(0, 0, h);
      
      // Sample points, using center value for out-of-bounds points
      const float fx_plus = sfSampler.sampleAndMap(P_x_plus).w;
      const float fx_minus = sfSampler.sampleAndMap(P_x_minus).w;
      const float fy_plus = sfSampler.sampleAndMap(P_y_plus).w;
      const float fy_minus = sfSampler.sampleAndMap(P_y_minus).w;
      const float fz_plus = sfSampler.sampleAndMap(P_z_plus).w;
      const float fz_minus = sfSampler.sampleAndMap(P_z_minus).w;

      const vec3f grad = vec3f(
          (fx_plus - fx_minus) / (2 * h),
          (fy_plus - fy_minus) / (2 * h),
          (fz_plus - fz_minus) / (2 * h));

      return normalize(grad);
    }
#endif

}
