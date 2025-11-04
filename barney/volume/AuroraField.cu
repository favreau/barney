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

#include "barney/volume/AuroraField.h"
#include "barney/Context.h"
#include "barney/volume/MCAccelerator.h"
#include "rtcore/ComputeInterface.h"

namespace BARNEY_NS {

  RTC_IMPORT_USER_GEOM(/*file*/AuroraField,/*name*/AuroraMC,
                       /*geomtype device data */
                       MCVolumeAccel<AuroraSampler>::DD,false,false);
  RTC_IMPORT_COMPUTE3D(AuroraField_computeMCs);

  AuroraField::PLD *AuroraField::getPLD(Device *device) 
  { return &perLogical[device->contextRank()]; } 

  /*! how many cells (in each dimension) will go into a macro
      cell for aurora field volumes */
  enum { cellsPerMC = 8 };

  /*! compute kernel that computes macro-cell information for Aurora volumes */
  struct AuroraField_ComputeMCs {
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
      
      // Sample aurora field values in a grid pattern within this macro cell
      vec3f mcOrigin = mcGrid.gridOrigin + vec3f(mcID) * mcGrid.gridSpacing;
      vec3f mcSize = mcGrid.gridSpacing;
      
      for (int iiz=0; iiz<=cellsPerMC; iiz++)
        for (int iiy=0; iiy<=cellsPerMC; iiy++)
          for (int iix=0; iix<=cellsPerMC; iix++) {
            vec3f samplePos = mcOrigin + vec3f(iix, iiy, iiz) * (mcSize / float(cellsPerMC));
            float fieldValue = auroraSampler.sample(samplePos);
            scalarRange.extend(fieldValue);
          }
          
      int mcIdx = mcID.x + mcGrid.dims.x*(mcID.y+mcGrid.dims.y*(mcID.z));
      mcGrid.scalarRanges[mcIdx] = scalarRange;
    }
#endif      
    /* kernel ARGS */
    MCGrid::DD mcGrid;
    AuroraSampler::DD auroraSampler;
  };

  // ==================================================================
  AuroraField::AuroraField(Context *context,
                          const DevGroup::SP &devices)
    : ScalarField(context,devices)
  {
    perLogical.resize(devices->numLogical);
    for (auto device : *devices)
      getPLD(device)->computeMCs
        = createCompute_AuroraField_computeMCs(device->rtc);
  }

  MCGrid::SP AuroraField::buildMCs()
  {
    if (mcGrid) return mcGrid;

    // Aurora extends from altitudeMin to altitudeMax radius
    float totalRadius = altitudeMax;
    vec3f boxMin = -vec3f(totalRadius);
    vec3f boxMax = vec3f(totalRadius);

    worldBounds = box3f(boxMin, boxMax);

    vec3f boxSize = boxMax - boxMin;
    float cellSize = totalRadius * 2.0f / static_cast<float>(cellsPerMC);
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
      auto tempSampler = std::make_shared<AuroraSampler>(this);
      AuroraField_ComputeMCs args = {
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

  AuroraSampler::DD AuroraSampler::getDD(Device *device)
  {
    DD dd;
    dd.intensity = sf->intensity;
    dd.waveFrequency = sf->waveFrequency;
    dd.waveAmplitude = sf->waveAmplitude;
    dd.time = sf->time;
    dd.altitudeMin = sf->altitudeMin;
    dd.altitudeMax = sf->altitudeMax;
    dd.thickness = sf->thickness;
    dd.turbulence = sf->turbulence;
    dd.numCurtains = sf->numCurtains;
    dd.magneticLatitude = sf->magneticLatitude;
    return dd;
  }

  bool AuroraField::set1f(const std::string &member, const float &value)
  {
    if (member == "intensity") {
      intensity = value;
      return true;
    }
    if (member == "waveFrequency") {
      waveFrequency = value;
      return true;
    }
    if (member == "waveAmplitude") {
      waveAmplitude = value;
      return true;
    }
    if (member == "time") {
      time = value;
      return true;
    }
    if (member == "altitudeMin") {
      altitudeMin = value;
      return true;
    }
    if (member == "altitudeMax") {
      altitudeMax = value;
      return true;
    }
    if (member == "thickness") {
      thickness = value;
      return true;
    }
    if (member == "turbulence") {
      turbulence = value;
      return true;
    }
    if (member == "numCurtains") {
      numCurtains = value;
      return true;
    }
    if (member == "magneticLatitude") {
      magneticLatitude = value;
      return true;
    }
    return ScalarField::set1f(member,value);
  }

  bool AuroraField::setObject(const std::string &member, const Object::SP &value)
  {
    return ScalarField::setObject(member,value);
  }

  void AuroraField::commit()
  {
    // Validate parameters
    intensity = max(0.0f, intensity);
    waveFrequency = max(0.1f, waveFrequency);
    waveAmplitude = max(0.0f, waveAmplitude);
    
    if (altitudeMax <= altitudeMin) {
      altitudeMin = 0.91f;
      altitudeMax = 0.94f;
    }
    
    thickness = max(0.0001f, thickness);
    turbulence = clamp(turbulence, 0.0f, 1.0f);
    numCurtains = max(1.0f, numCurtains);
    magneticLatitude = clamp(magneticLatitude, -90.0f, 90.0f);
    
    // Invalidate macro cells so they get rebuilt with new parameters
    mcGrid.reset();
  }

  AuroraField::DD AuroraField::getDD(Device *device)
  {
    AuroraField::DD dd;
    (ScalarField::DD &)dd = ScalarField::getDD(device);
    dd.intensity = intensity;
    dd.waveFrequency = waveFrequency;
    dd.waveAmplitude = waveAmplitude;
    dd.time = time;
    dd.altitudeMin = altitudeMin;
    dd.altitudeMax = altitudeMax;
    dd.thickness = thickness;
    dd.turbulence = turbulence;
    dd.numCurtains = numCurtains;
    dd.magneticLatitude = magneticLatitude;
    return dd;
  }

  VolumeAccel::SP AuroraField::createAccel(Volume *volume)
  {
    auto sampler = std::make_shared<AuroraSampler>(this);
    return std::make_shared<MCVolumeAccel<AuroraSampler>>
      (volume,
       createGeomType_AuroraMC,
       sampler);
  }

  RTC_EXPORT_COMPUTE3D(AuroraField_computeMCs,AuroraField_ComputeMCs);

} // namespace BARNEY_NS

