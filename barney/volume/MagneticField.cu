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

#include "barney/Context.h"
#include "barney/volume/MagneticField.h"
#include "barney/common/Texture.h"
#include "rtcore/ComputeInterface.h"

namespace BARNEY_NS {

  RTC_IMPORT_USER_GEOM(/*file*/MagneticField,/*name*/MagneticMC,
                       /*geomtype device data */
                       MCVolumeAccel<MagneticSampler>::DD,false,false);
  RTC_IMPORT_COMPUTE3D(MagneticField_computeMCs);

  MagneticField::PLD *MagneticField::getPLD(Device *device) 
  { return &perLogical[device->contextRank()]; } 

  /*! how many cells (in each dimension) will go into a macro
      cell for magnetic field volumes */
  enum { cellsPerMC = 8 };

  /*! compute kernel that computes macro-cell information for Magnetic field volumes */
  struct MagneticField_ComputeMCs {
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
      
      // Sample magnetic field values in a grid pattern within this macro cell
      vec3f mcOrigin = mcGrid.gridOrigin + vec3f(mcID) * mcGrid.gridSpacing;
      vec3f mcSize = mcGrid.gridSpacing;
      
      for (int iiz=0; iiz<=cellsPerMC; iiz++)
        for (int iiy=0; iiy<=cellsPerMC; iiy++)
          for (int iix=0; iix<=cellsPerMC; iix++) {
            vec3f samplePos = mcOrigin + vec3f(iix, iiy, iiz) * (mcSize / float(cellsPerMC));
            float fieldStrength = magneticSampler.sample(samplePos);
            scalarRange.extend(fieldStrength);
          }
          
      int mcIdx = mcID.x + mcGrid.dims.x*(mcID.y+mcGrid.dims.y*(mcID.z));
      mcGrid.scalarRanges[mcIdx] = scalarRange;
    }
#endif      
    /* kernel ARGS */
    MCGrid::DD mcGrid;
    MagneticSampler::DD magneticSampler;
  };
  
  MagneticField::MagneticField(Context *context,
                              const DevGroup::SP &devices)
    : ScalarField(context,devices)
  {
    perLogical.resize(devices->numLogical);
    for (auto device : *devices)
      getPLD(device)->computeMCs
        = createCompute_MagneticField_computeMCs(device->rtc);
  }

  MCGrid::SP MagneticField::buildMCs() 
  {
    if (mcGrid) return mcGrid;
    
    // Earth's magnetosphere extends to about 3 Earth radii
    float magnetosphereRadius = 3.0f;
    vec3f boxMin = -vec3f(magnetosphereRadius);
    vec3f boxMax = vec3f(magnetosphereRadius);
    
    worldBounds = box3f(boxMin, boxMax);
    
    vec3f boxSize = boxMax - boxMin;
    float cellSize = magnetosphereRadius * 2.0f / static_cast<float>(cellsPerMC);
    vec3i mcDims = vec3i(ceil(boxSize.x / cellSize), ceil(boxSize.y / cellSize), ceil(boxSize.z / cellSize));
    mcDims = max(mcDims, vec3i(4)); // Minimum grid size
    
    mcGrid = std::make_shared<MCGrid>(devices);
    mcGrid->resize(mcDims);
    vec3ui blockSize(4);
    vec3ui numBlocks = divRoundUp(vec3ui(mcDims),blockSize);
    mcGrid->gridOrigin = boxMin;
    mcGrid->gridSpacing = boxSize / vec3f(mcDims);
    
    for (auto device : *devices) {
      PLD *pld = getPLD(device);
      auto tempSampler = std::make_shared<MagneticSampler>(this);
      MagneticField_ComputeMCs args = {
        mcGrid->getDD(device),
        tempSampler->getDD(device)
      };
      pld->computeMCs->launch(numBlocks,blockSize,
                              &args);
    }
    
    for (auto device : *devices)
      device->sync();
    
    return mcGrid;
  }

  MagneticSampler::DD MagneticSampler::getDD(Device *device)
  {
    DD dd;
    dd.equatorStrength = sf->equatorStrength;
    dd.poleStrength = sf->poleStrength;
    dd.dipoleTilt = sf->dipoleTilt;
    return dd;
  }

  bool MagneticField::set1f(const std::string &member, const float &value)
  {
    if (member == "equatorStrength") {
      equatorStrength = value;
      return true;
    }
    if (member == "poleStrength") {
      poleStrength = value;
      return true;
    }
    if (member == "dipoleTilt") {
      dipoleTilt = value;
      return true;
    }
    return ScalarField::set1f(member,value);
  }

  bool MagneticField::setObject(const std::string &member, const Object::SP &value)
  {
    return ScalarField::setObject(member,value);
  }

  void MagneticField::commit()
  {
    // Validate parameters
    equatorStrength = max(0.0f, equatorStrength);
    poleStrength = max(0.0f, poleStrength);
    dipoleTilt = clamp(dipoleTilt, -90.0f, 90.0f);
    
    // Invalidate macro cells so they get rebuilt with new parameters
    mcGrid.reset();
  }

  MagneticField::DD MagneticField::getDD(Device *device)
  {
    MagneticField::DD dd;
    (ScalarField::DD &)dd = ScalarField::getDD(device);
    dd.equatorStrength = equatorStrength;
    dd.poleStrength = poleStrength;
    dd.dipoleTilt = dipoleTilt;
    return dd;
  }

  VolumeAccel::SP MagneticField::createAccel(Volume *volume)
  {
    auto sampler = std::make_shared<MagneticSampler>(this);
    return std::make_shared<MCVolumeAccel<MagneticSampler>>
      (volume,
       createGeomType_MagneticMC,
       sampler);
  }

  RTC_EXPORT_COMPUTE3D(MagneticField_computeMCs,MagneticField_ComputeMCs);

}
