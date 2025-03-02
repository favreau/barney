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

#include "rtcore/common/rtcore-common.h"
#include "embree4/rtcore.h"

#define __rtc_device /* ignore - for embree device we use all cpu */
#define __rtc_both /* ignore - for embree device we use all cpu */
    
namespace rtc {
  namespace embree {

    using namespace owl::common;    
    
    // ------------------------------------------------------------------
    // cuda vector types - we may NOT use the cuda vector types EVEN
    // IF CUDA IS AVAILABLE, because the cuda vector types have
    // alignemnt, and these here do not - meaning that if some files
    // were to get compiled by nvcc and others by gcc/msvc we get
    // nasty differences is interpreting the same types in differnt ways
    // ------------------------------------------------------------------
    struct float3 { float x; float y; float z; };
    struct float4 { float x; float y; float z; float w; };

    inline vec3f load(const ::rtc::embree::float3 &v) { return (const vec3f&)v; }
    inline vec4f load(const ::rtc::embree::float4 &v) { return (const vec4f&)v; }
    
  }
}

