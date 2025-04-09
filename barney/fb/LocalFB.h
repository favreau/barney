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

#include "barney/Context.h"
#include "barney/fb/FrameBuffer.h"

namespace BARNEY_NS {

  /*! implements a frame buffer for a single 'local' node where all
      GPUs can be seen/accessed directly on the current node. */
  struct LocalFB : public FrameBuffer {
    typedef std::shared_ptr<LocalFB> SP;

    LocalFB(Context *context,
            const DevGroup::SP &devices);
    virtual ~LocalFB();
    
    /*! pretty-printer for printf-debugging */
    std::string toString() const override
    { return "LocalFB{}"; }

    /*! resize frame buffer to given number of pixels and the
        indicated types of channels; color will only ever get queries
        in 'colorFormat'. Channels is a bitmask compoosed of
        or'ed-together BN_FB_xyz channel flags; only those bits that
        are set may get queried by the application (ie those that are
        not set do not have to be stored or even computed */
    void resize(BNDataType colorFormat,
                vec2i size,
                uint32_t channels) override;

    /*! gather color (and optionally, if not null) linear normal, from
        all GPUs (and ranks). lienarColor and lienarNormal are
        device-writeable 2D linear arrays of numPixel size;
        linearcolor may be null. */
    void gatherColorChannel(/*float4 or rgba8*/void *linearColor,
                            BNDataType gatherType,
                            vec3f *linearNormal) override;
      
    /*! read one of the auxiliary (not color or normal) buffers into
      the given (device-writeable) staging area; this will at the
      least incur some reformatting from tiles to linear (if local
      node), possibly some gpu-gpu transfer (local node w/ more than
      one gpu) and possibly some mpi communication (distFB) */
    void gatherAuxChannel(BNFrameBufferChannel channel) override;
    void writeAuxChannel(void *stagingArea,
                          BNFrameBufferChannel channel) override;

    struct {
      /*! _all_ tile descriptors across all GPUs - either all GPUs in
        single node (if run non-mpi) or across all nodes */
      TileDesc *tileDescs       = 0;
      int       sumTiles = 0;
    } onOwner;
    
  };

}
