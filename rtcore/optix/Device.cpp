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

#include "rtcore/optix/Device.h"
#include "rtcore/optix/Denoiser.h"
#include "rtcore/optix/Buffer.h"
#include "rtcore/optix/Geom.h"
#include "rtcore/optix/Group.h"
#include <optix.h>
#include <optix_function_table.h>
#include <optix_stubs.h>

namespace rtc {
  namespace optix {
    
    Device::Device(int physicalGPU)
      : cuda_common::Device(physicalGPU)
    {
      owl = owlContextCreate(&physicalGPU,1);
    }

    Device::~Device()
    {
      destroy();
    }

    Denoiser *Device::createDenoiser()
    {
#if !OPTIX_DISABLE_DENOISING && OPTIX_VERSION >= 80000
      return new Optix8Denoiser(this);
#else
      // we only support optix 8 denoiser
      return nullptr;
#endif
    }

    void Device::destroy()
    {
      if (owl) owlContextDestroy(owl);
      owl = 0;
    }



    void Device::sync()
    {
      cuda_common::Device::sync();
      for (auto s : activeTraceStreams) {
        cudaStreamSynchronize(s);
      }
      activeTraceStreams.clear();
    }
    
    
    // ==================================================================
    // rtcore/pipeline stuff
    // ==================================================================
    void Device::buildPipeline() 
    {
      if (!programsDirty) return;
      owlBuildPipeline(owl);
    }
      
    void Device::buildSBT() 
    {
      owlBuildSBT(owl);
    }
      
    

    // ==================================================================
    // buffer
    // ==================================================================

    void Device::freeBuffer(Buffer *buffer) 
    {
      delete buffer;
    }
    
    Buffer *Device::createBuffer(size_t numBytes,
                                      const void *initValues) 
    {
      return new Buffer(this,numBytes,initValues);
    }


    // ==================================================================
    // geom
    // ==================================================================

    void Device::freeGeom(Geom *geom)
    { assert(geom); delete geom; }
    
    // ==================================================================
    // geomtype
    // ==================================================================
    
    void Device::freeGeomType(GeomType *gt)
    { assert(gt); delete gt; }
    
    // ==================================================================
    // kernels
    // ==================================================================


    TraceKernel2D::TraceKernel2D(Device *device,
                                 const std::string &ptxCode,
                                 const std::string &kernelName,
                                 size_t sizeOfLP)
      : device(device)
    {
      OWLVarDecl rg_args[]
        = {
        { nullptr }
      };
      mod = owlModuleCreate(device->owl,ptxCode.c_str());
      rg = owlRayGenCreate(device->owl,mod,
                           kernelName.c_str(),
                           0,rg_args,-1);
      owlBuildPrograms(device->owl);
      
      OWLVarDecl lp_args[]
        = {
        { "raw", (OWLDataType)(OWL_USER_TYPE_BEGIN+sizeOfLP), 0 },
        { nullptr }
      };
      lp = owlParamsCreate(device->owl,sizeOfLP,lp_args,-1);
      lpStream = owlParamsGetCudaStream(lp,0);
      device->programsDirty = true;
    }
    
    void TraceKernel2D::launch(vec2i dims,
                               const void *kernelData)
    {
      SetActiveGPU forDuration(device);
      BARNEY_CUDA_CALL(StreamSynchronize(/*inherited!*/device->stream));
      
      owlParamsSetRaw(lp,"raw",kernelData,0);
      if (dims.x > 0 && dims.y > 0) {
        owlAsyncLaunch2D(rg,dims.x,dims.y,lp);
        device->activeTraceStreams.push_back(lpStream);
      }
    }
    
    // ==================================================================
    // groups
    // ==================================================================
    void Device::freeGroup(Group *group)
    {
      delete group;
    }

    Group *
    Device::createTrianglesGroup(const std::vector<Geom *> &geoms)
    {
      std::vector<OWLGeom> owlGeoms;
      for (auto geom : geoms)
        owlGeoms.push_back(((Geom *)geom)->owl);
      OWLGroup g = owlTrianglesGeomGroupCreate(owl,
                                               owlGeoms.size(),
                                               owlGeoms.data());
      return new Group(this,g);
    }

    Group *
    Device::createUserGeomsGroup(const std::vector<Geom *> &geoms)
    {
      std::vector<OWLGeom> owlGeoms;
      for (auto geom : geoms)
        owlGeoms.push_back(((Geom *)geom)->owl);
      OWLGroup g = owlUserGeomGroupCreate(owl,
                                          owlGeoms.size(),
                                          owlGeoms.data());
      return new Group(this,g);
    }

    Group *
    Device::createInstanceGroup(const std::vector<Group *> &groups,
                                  const std::vector<affine3f> &xfms)
    {
      std::vector<OWLGroup> owls;
      for (auto group : groups)
        owls.push_back(((Group *)group)->owl);

      OWLGroup g
        = owlInstanceGroupCreate(owl,
                                 owls.size(),
                                 owls.data(),
                                 nullptr,
                                 (const float *)xfms.data());
      Group *gg = new Group(this,g);
      return gg;
    }

  }
}

