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

#include "barney/Context.h"
#include "barney/volume/CloudField.h"
#include "barney/common/Texture.h"
#include "rtcore/ComputeInterface.h"

namespace BARNEY_NS {

  RTC_IMPORT_USER_GEOM(/*file*/CloudField,/*name*/CloudMC,
                       /*geomtype device data */
                       MCVolumeAccel<CloudSampler>::DD,false,false);
  RTC_IMPORT_COMPUTE3D(CloudField_computeMCs);

  CloudField::CLD *CloudField::getCLD(Device *device) 
  { return &perLogical[device->contextRank()]; } 

  /*! how many cells (in each dimension) will go into a macro
      cell for spherical volumes */
  enum { cellsPerMC = 8 };

  /*! compute kernel that computes macro-cell information for Cloud volumes */
  struct CloudField_ComputeMCs {
#if RTC_DEVICE_CODE
    /* kernel CODE */
    inline __rtc_device void run(const rtc::ComputeInterface &rtCore)
    {
      vec3i mcID
        = vec3i(rtCore.getThreadIdx())
        + vec3i(rtCore.getBlockIdx())
        * vec3i(rtCore.getBlockDim());
      if (mcID.x >= mcGrid.dims.x) return;
      if (mcID.y >= mcGrid.dims.y) return;
      if (mcID.z >= mcGrid.dims.z) return;
        
      range1f scalarRange;
      
      // Sample density values in a grid pattern within this macro cell
      vec3f mcOrigin = mcGrid.gridOrigin + vec3f(mcID) * mcGrid.gridSpacing;
      vec3f mcSize = mcGrid.gridSpacing;
      
      for (int iiz=0; iiz<=cellsPerMC; iiz++)
        for (int iiy=0; iiy<=cellsPerMC; iiy++)
          for (int iix=0; iix<=cellsPerMC; iix++) {
            vec3f samplePos = mcOrigin + vec3f(iix, iiy, iiz) * (mcSize / float(cellsPerMC));
            float density = cloudSampler.sample(samplePos);
            scalarRange.extend(density);
          }
          
      int mcIdx = mcID.x + mcGrid.dims.x*(mcID.y+mcGrid.dims.y*(mcID.z));
      mcGrid.scalarRanges[mcIdx] = scalarRange;
    }
#endif      
    /* kernel ARGS */
    MCGrid::DD mcGrid;
    CloudSampler::DD cloudSampler;
  };
  
  
  CloudField::CloudField(Context *context,
                         const DevGroup::SP &devices)
    : ScalarField(context,devices)
  {
    perLogical.resize(devices->numLogical);
    for (auto device : *devices)
      getCLD(device)->computeMCs
        = createCompute_CloudField_computeMCs(device->rtc);
  }

  void CloudField::buildMCs(MCGrid &mcGrid) 
  {
    // Create macro cell grid that covers the spherical volume
    // The sphere is centered at origin with radius planetRadius + atmosphereThickness
    float totalRadius = planetRadius + atmosphereThickness;
    vec3f boxMin = -vec3f(totalRadius);
    vec3f boxMax = vec3f(totalRadius);
    
    worldBounds = box3f(boxMin, boxMax);
    
    vec3f boxSize = boxMax - boxMin;
    float cellSize = totalRadius * 2.0f / static_cast<float>(cellsPerMC);
    vec3i mcDims = vec3i(ceil(boxSize.x / cellSize), ceil(boxSize.y / cellSize), ceil(boxSize.z / cellSize));
    mcDims = max(mcDims, vec3i(4)); // Minimum grid size
    
    mcGrid.resize(mcDims);
    vec3ui blockSize(4);
    vec3ui numBlocks = divRoundUp(vec3ui(mcDims),blockSize);
    mcGrid.gridOrigin = boxMin;
    mcGrid.gridSpacing = boxSize / vec3f(mcDims);
    
    for (auto device : *devices) {
      CLD *cld = getCLD(device);
      auto tempSampler = std::make_shared<CloudSampler>(this);
      CloudField_ComputeMCs args = {
        mcGrid.getDD(device),
        tempSampler->getDD(device)
      };
      cld->computeMCs->launch(numBlocks,blockSize,
                              &args);
    }
    for (auto device : *devices)
      device->sync();
  }
  
  CloudSampler::DD CloudSampler::getDD(Device *device)
  {
    DD dd;
    
    // Set texture object and dimensions
    if (sf->cloudData) {
      dd.cloudDataTex = sf->cloudData->getDD(device);
      dd.textureDims = sf->cloudTextureData->dims; // Get texture dimensions for coordinate calculation
    }
    
    // Set sphere parameters
    dd.planetRadius = sf->planetRadius;
    dd.atmosphereThickness = sf->atmosphereThickness;
    
    return dd;
  }
  
  VolumeAccel::SP CloudField::createAccel(Volume *volume) 
  {
    auto sampler = std::make_shared<CloudSampler>(this);
    return std::make_shared<MCVolumeAccel<CloudSampler>>
      (volume,
       createGeomType_CloudMC,
       sampler);
  }
  
  // ==================================================================
  bool CloudField::set1f(const std::string &member,
                         const float &value) 
  {
    if (member == "planetRadius") {
      planetRadius = value;
      return true;
    }
    if (member == "atmosphereThickness") {
      atmosphereThickness = value;
      return true;
    }
    return false;
  }

  // ==================================================================
  bool CloudField::setObject(const std::string &member,
                             const Object::SP &value) 
  {
    BNTextureAddressMode addressModes[3] = {
      BN_TEXTURE_CLAMP, BN_TEXTURE_CLAMP, BN_TEXTURE_CLAMP
    };
    
    if (member == "cloudData") {
      cloudTextureData = value->as<TextureData>();
      cloudData = std::make_shared<Texture>((Context*)context, cloudTextureData,
                                           BN_TEXTURE_LINEAR,
                                           addressModes,
                                           BN_COLOR_SPACE_LINEAR);
      return true;
    }
    return false;
  }

  // ==================================================================
  void CloudField::commit() 
  {
    float totalRadius = planetRadius + atmosphereThickness;
    worldBounds.lower = -vec3f(totalRadius);
    worldBounds.upper = vec3f(totalRadius);
  }

  CloudField::DD CloudField::getDD(Device *device)
  {
    DD dd;
    dd.worldBounds = worldBounds;
    dd.planetRadius = planetRadius;
    dd.atmosphereThickness = atmosphereThickness;
    return dd;
  }
  
  RTC_EXPORT_COMPUTE3D(CloudField_computeMCs,CloudField_ComputeMCs);
} 