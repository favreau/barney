// ======================================================================== //
// Copyright 2023-2023 Ingo Wald                                            //
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
#include "rtcore/AppInterface.h"
#include "barney/WorkerTopo.h"

namespace BARNEY_NS {
  
  struct TiledFB;
  struct RayQueue;
  
  typedef rtc::GeomType *(*GeomTypeCreationFct)(rtc::Device *device);
  
  struct GeomTypeRegistry {
    GeomTypeRegistry(rtc::Device *device);
    rtc::GeomType *get(GeomTypeCreationFct callback);
    std::map<GeomTypeCreationFct,rtc::GeomType *> geomTypes;

    rtc::Device *const device;    
  };

  /*! mpi-like descriptor of a group of peers, enumerating them by
      'rank' (=0,1,2...size-1) and giving total num peers in 'size' */
  struct PeerGroup {
    int rank = -1;
    int size = -1;
  };
  
  struct Device {
    Device(rtc::Device *rtc,
           const WorkerTopo *topo,
           int localRank);

    int worldRank() const;

    int globalRank() const;
    int globalSize() const;

    int localRank() const;
    int localSize() const;

    // DEPRECATED!
    int contextRank() const;
    
    // int globalRank() const { return allGPUsGlobally.rank; }
    // int globalSize() const { return allGPUsGlobally.size; }

    // int localRank() const { return allGPUsLocally.rank; }
    // int localSize() const { return allGPUsLocally.size; }

    // // DEPRECATED!
    // int contextRank() const { return localRank(); }
    
    // int                globalRank = -1;
    // int                globalSize = -1;
    
    // /*! describes this device's island's place in the world */
    // PeerGroup islandInWorld;

    // /*! describes this device's place within the island/cycle that it
    //     is in */
    // PeerGroup gpuInIsland;
    
    void sync() { rtc->sync(); }
    
    /* for ray queue cycling - who to cycle with */
    // struct {
    //   int sendWorkerRank  = -1;
    //   int sendWorkerLocal = -1;
    //   int recvWorkerRank  = -1;
    //   int recvWorkerLocal = -1;
    // } rqs;

    int  setActive() const { return rtc->setActive(); }
    void restoreActive(int old) const  { rtc->restoreActive(old); }
    void syncPipelineAndSBT();
    
    bool sbtDirty = true;
    
    GeomTypeRegistry geomTypes;
    rtc::Device *const rtc;
    // rtc::ComputeKernel1D *generateRays = 0;
    // rtc::ComputeKernel1D *shadeRays = 0;
    
    rtc::TraceKernel2D *traceRays = 0;
    RayQueue     *rayQueue = 0;


    /*! the _global_ device ID within the worker topo */
    int const _localRank;
    int const _globalRank;
    const WorkerTopo *const topo;
  };
  
  /*! stolen from owl/Device: helper class that will set the
    active cuda device (to the device associated with a given
    Context::DeviceData) for the duration fo the lifetime of this
    class, and resets it to whatever it was after class dies */
  struct SetActiveGPU {
    inline SetActiveGPU(const Device *device)
      : savedDevice(device)
    { assert(device); savedActiveDeviceID = device->setActive(); }
    
    inline ~SetActiveGPU()
    { savedDevice->restoreActive(savedActiveDeviceID); }
  private:
    int savedActiveDeviceID = -1;
    const Device *const savedDevice;
  };

  /*! a group of devices that need to share in "something".
    in practive, this is either one of:

    a) the list of devices in a given local model slot

    b) a list of all devices in the local context; or

    c) a single device (eg, the one that does final frame buffer
    assembly and/or denoisign

    In the first case the lmsIdx is the local index of that model slot,
    in the other cases it is '-1'
  */
  struct DevGroup : public std::vector<Device*> {
    typedef std::shared_ptr<DevGroup> SP;
    
    DevGroup(const std::vector<Device*> &devices,
             int numLogical);

    Device *get(int idx) { return (*this)[idx]; }
    
      /*! *TOTAL* number of logical devices in the context;
      *NOT* how many devices there are in this group. */
    int const numLogical;
  };
  
}
