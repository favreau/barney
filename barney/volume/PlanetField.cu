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
#include "barney/volume/PlanetField.h"
#include "barney/common/Texture.h"
#include "rtcore/ComputeInterface.h"

namespace BARNEY_NS {

  RTC_IMPORT_USER_GEOM(/*file*/PlanetField,/*name*/PlanetMC,
                       /*geomtype device data */
                       MCVolumeAccel<PlanetSampler>::DD,false,false);
  RTC_IMPORT_COMPUTE3D(PlanetField_computeMCs);

  PlanetField::PLD *PlanetField::getPLD(Device *device) 
  { return &perLogical[device->contextRank()]; } 

  /*! how many cells (in each dimension) will go into a macro
      cell for spherical volumes */
  enum { cellsPerMC = 8 };

  /*! compute kernel that computes macro-cell information for Planet volumes */
  struct PlanetField_ComputeMCs {
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
            float density = planetSampler.sample(samplePos);
            scalarRange.extend(density);
          }
          
      int mcIdx = mcID.x + mcGrid.dims.x*(mcID.y+mcGrid.dims.y*(mcID.z));
      mcGrid.scalarRanges[mcIdx] = scalarRange;
    }
#endif      
    /* kernel ARGS */
    MCGrid::DD mcGrid;
    PlanetSampler::DD planetSampler;
  };
  
  PlanetField::PlanetField(Context *context,
                         const DevGroup::SP &devices)
    : ScalarField(context,devices)
  {
    perLogical.resize(devices->numLogical);
    for (auto device : *devices)
      getPLD(device)->computeMCs
        = createCompute_PlanetField_computeMCs(device->rtc);
  }

  void PlanetField::buildMCs(MCGrid &mcGrid) 
  {
    float totalRadius = 1.f;
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
      PLD *pld = getPLD(device);
      auto tempSampler = std::make_shared<PlanetSampler>(this);
      PlanetField_ComputeMCs args = {
        mcGrid.getDD(device),
        tempSampler->getDD(device)
      };
      pld->computeMCs->launch(numBlocks,blockSize,
                              &args);
    }
    for (auto device : *devices)
      device->sync();
  }
  
  PlanetSampler::DD PlanetSampler::getDD(Device *device)
  {
    DD dd;
    
    // Set texture objects
    if (sf->elevationMap)
      dd.elevationTex = sf->elevationMap->getDD(device);
    if (sf->diffuseMap)
      dd.diffuseTex = sf->diffuseMap->getDD(device);
    if (sf->normalMap)
      dd.normalTex = sf->normalMap->getDD(device);
    
    // Set sphere parameters
    dd.planetRadius = sf->planetRadius;
    dd.elevationScale = sf->elevationScale;
    
    return dd;
  }
  
  VolumeAccel::SP PlanetField::createAccel(Volume *volume) 
  {
    auto sampler = std::make_shared<PlanetSampler>(this);
    return std::make_shared<MCVolumeAccel<PlanetSampler>>
      (volume,
       createGeomType_PlanetMC,
       sampler);
  }
  
  // ==================================================================
  bool PlanetField::set1f(const std::string &member,
                         const float &value) 
  {
    if (member == "planetRadius") {
      planetRadius = value;
      return true;
    }
    if (member == "elevationScale") {
      elevationScale = value;
      return true;
    }
    return false;
  }

  // ==================================================================
  bool PlanetField::setObject(const std::string &member,
                             const Object::SP &value) 
  {
    BNTextureAddressMode addressModes[3] = {
      BN_TEXTURE_WRAP, BN_TEXTURE_CLAMP, BN_TEXTURE_CLAMP
    };
    
    if (member == "elevationMap") {
      elevationData = value->as<TextureData>();
      elevationMap = std::make_shared<Texture>((Context*)context, elevationData,
                                              BN_TEXTURE_LINEAR,
                                              addressModes,
                                              BN_COLOR_SPACE_LINEAR);
      return true;
    }
    if (member == "diffuseMap") {
      diffuseData = value->as<TextureData>();
      diffuseMap = std::make_shared<Texture>((Context*)context, diffuseData,
                                            BN_TEXTURE_LINEAR,
                                            addressModes,
                                            BN_COLOR_SPACE_LINEAR);
      return true;
    }
    if (member == "normalMap") {
      normalData = value->as<TextureData>();
      normalMap = std::make_shared<Texture>((Context*)context, normalData,
                                           BN_TEXTURE_LINEAR,
                                           addressModes,
                                           BN_COLOR_SPACE_LINEAR);
      return true;
    }
    return false;
  }

  // ==================================================================
  void PlanetField::commit() 
  {
    const float totalRadius = planetRadius + elevationScale;
    worldBounds.lower = -vec3f(totalRadius);
    worldBounds.upper = vec3f(totalRadius);
  }

  PlanetField::DD PlanetField::getDD(Device *device)
  {
    DD dd;
    dd.worldBounds = worldBounds;
    dd.planetRadius = planetRadius;
    dd.elevationScale = elevationScale;
    return dd;
  }
  
  RTC_EXPORT_COMPUTE3D(PlanetField_computeMCs,PlanetField_ComputeMCs);
} 