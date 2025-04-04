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

#include "rtcore/cudaCommon/cuda-common.h"

namespace rtc {
  namespace cuda_common {

    using rtc::TextureObject;
    
    struct Device;
    struct Texture;
    struct TextureData;
    
    struct SetActiveGPU {
      inline SetActiveGPU(const Device *device);
      inline SetActiveGPU(int gpuID);
      inline ~SetActiveGPU();
    private:
      int savedActiveDeviceID = -1;
      // const Device *const savedDevice;
    };

    /*! base class for cuda-based device(s) - unlike optix/device and
        embree/device this is NOT a full device as it lacks trace
        capability. will be subclassed by otpix device (which adds
        optix-based trace interface, and at some later time a
        dedicated cuda trace device */
    struct Device {
      Device(int physicalGPU)
        : physicalID(physicalGPU)
      {
        int saved = setActive();
        BARNEY_CUDA_CALL(StreamCreate(&stream));
        restoreActive(saved);
      }
      
      void copyAsync(void *dst, const void *src, size_t numBytes);
      void copy(void *dst, const void *src, size_t numBytes)
      { copyAsync(dst,src,numBytes); sync(); }
      void *allocHost(size_t numBytes);
      void freeHost(void *mem);
      void memsetAsync(void *mem,int value, size_t size);
      void *allocMem(size_t numBytes);
      void freeMem(void *mem);
      void sync();
      
      /*! sets this gpu as active, and returns physical ID of GPU that
        was active before */
      int setActive() const;
      
      /*! restores the gpu whose ID was previously returend by setActive() */
      void restoreActive(int oldActive) const;

      TextureData *createTextureData(vec3i dims,
                                     rtc::DataType format,
                                     const void *texels);
      
      void freeTextureData(TextureData *);
      void freeTexture(Texture *);
      
      cudaStream_t stream;
      int const physicalID;
    };

    /*! enable peer access between these gpus, and return truea if
        successful, else if at least one pair does not work */
    bool enablePeerAccess(const std::vector<int> &gpuIDs);
    
    inline SetActiveGPU::SetActiveGPU(const Device *device)
    {
      if (device) 
        savedActiveDeviceID = device->setActive();
      else 
        BARNEY_CUDA_CHECK(cudaGetDevice(&savedActiveDeviceID));
    }

    inline SetActiveGPU::SetActiveGPU(int gpuID)
      // : savedDevice(null)
    {
      BARNEY_CUDA_CHECK(cudaGetDevice(&savedActiveDeviceID));
      BARNEY_CUDA_CHECK(cudaSetDevice(gpuID));
    }

    inline SetActiveGPU::~SetActiveGPU()
    {
      BARNEY_CUDA_CALL_NOTHROW(SetDevice(savedActiveDeviceID));
      // if (savedDevice) 
      //   savedDevice->restoreActive(savedActiveDeviceID);
      // else
        
    }
    
  }
}
