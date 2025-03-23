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

#include "barney/render/DG.h"

namespace BARNEY_NS {
  namespace render {
    namespace packedBSDF {

      /*! implements a homogenous phase function that scatters equally
          in all directions, with given average reflectance and
          color */
      struct Lambertian {
        inline Lambertian() = default;
        inline __rtc_device Lambertian(vec3f color, float avg_reflectance=1.f);

        inline __rtc_device
        float pdf(DG dg, vec3f wi, bool dbg) const;
        
        inline __rtc_device
        EvalRes eval(DG dg, vec3f wi, bool dbg) const;
        
        inline __rtc_device
        void scatter(ScatterResult &scatter,
                     const render::DG &dg,
                     Random &random,
                     bool dbg) const;
        
        rtc::float3 albedo;
      };

      inline __rtc_device
      float Lambertian::pdf(DG dg, vec3f wi, bool dbg) const
      { 
        vec3f N = dg.Ng;
        if (dot(wi,N) < 0.f) N = -N;
        
        float cosThetaI = max(dot(wi, N), 0.f);
        float pdf = cosineSampleHemispherePDF(cosThetaI);
        return pdf;
      }
        
      inline __rtc_device
      EvalRes Lambertian::eval(DG dg, vec3f wi, bool dbg) const
      {
        vec3f N = dg.Ng;
        if (dot(wi,N) < 0.f) N = -N;
        
        float cosThetaI = max(dot(wi, dg.Ns), 0.f);
        float pdf = cosineSampleHemispherePDF(cosThetaI);
        return EvalRes(rtc::load(albedo) * ONE_OVER_PI * cosThetaI,pdf);
      }

      /*! simple omnidirectional phase function - scatter into any
        random direction */
      inline __rtc_device
      void Lambertian::scatter(ScatterResult &scatter,
                          const render::DG &dg,
                          Random &random,
                          bool dbg) const
      {
        vec3f N = dg.Ng;
        if (dot(dg.wo,N) < 0.f) N = -N;
        
        vec2f s(random(),random());
        vec3f localDir = cosineSampleHemisphere(s);

        scatter.dir = owl::common::xfmVector(owl::common::frame(N),localDir);
        scatter.pdf = cosineSampleHemispherePDF(localDir);
        scatter.f_r  = rtc::load(albedo);
        scatter.type = ScatterResult::DIFFUSE;
      }
      
    }
  }
}

