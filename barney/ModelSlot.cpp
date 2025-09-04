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

#include "barney/ModelSlot.h"
#include "barney/GlobalModel.h"
#include "barney/common/Data.h"
#include "barney/common/Texture.h"
#include "barney/light/Light.h"
#include "barney/geometry/Geometry.h"

namespace BARNEY_NS {

  ModelSlot::PLD *ModelSlot::getPLD(Device *device)
  {
    assert(device);
    assert(device->contextRank() >= 0);
    assert(device->contextRank() < perLogical.size());
    return &perLogical[device->contextRank()];
  }

  ModelSlot::ModelSlot(GlobalModel *_model,
                       const DevGroup::SP &devices,
                       int slotID)
    : SlottedObject((Context *)_model->context,
                    devices),
      model(_model),
      slotID(slotID),
      slotContext(((Context *)_model->context)->getSlot(slotID)),
      world(std::make_shared<render::World>(slotContext))
  {
    perLogical.resize(devices->numLogical);
  }

  ModelSlot::~ModelSlot()
  {}
  
  void ModelSlot::setInstances(barney_api::Group **groups,
                               const affine3f *xfms,
                               int numUserInstances)
  {
    instances.groups.resize(numUserInstances);
    instances.xfms.resize(numUserInstances);
    for (int i=0;i<numUserInstances;i++) {
      auto g = groups[i];
      instances.groups[i]
        = g
        ? g->shared_from_this()->as<Group>()
        : Group::SP{};
    }
    std::copy(xfms,xfms+numUserInstances,instances.xfms.data());
    for (auto device : *devices) {
      device->sbtDirty = true;
      auto pld = getPLD(device);
      if (pld->instanceGroup) {
        device->rtc->freeGroup(pld->instanceGroup);
        pld->instanceGroup = 0;
      }
    }
  }

  void ModelSlot::setInstanceAttributes(const std::string &which,
                                        const PODData::SP &data)
  {
    // assert(which >= 0 && which < 5);
    if (which == "instID")
      world->instanceUserIDs = data;
    else if (which == "attribute0")
      world->instanceAttributes[0] = data;
    else if (which == "attribute1")
      world->instanceAttributes[1] = data;
    else if (which == "attribute2")
      world->instanceAttributes[2] = data;
    else if (which == "attribute3")
      world->instanceAttributes[3] = data;
    else if (which == "attribute4" || which == "color")
      world->instanceAttributes[4] = data;
    else
      std::cout << "#barney: un-recognized instance attribute '" << which << "'" << std::endl;
      ;
  }
  
  void ModelSlot::build()
  {
    std::vector<affine3f> rtcTransforms;
    EnvMapLight::SP envMapLight;

    // ==================================================================
    // generate all lights's "raw" data. note this is NOT per device
    // (yet), even though the use of 'DD's seems to imply so. this
    // should "eventually" be changed to something where the current
    // 'world' class gets merged into 'modelslot', and all light,
    // material, and texture data then live 'per logical device'
    // ==================================================================
    std::vector<QuadLight::DD> quadLights;
    std::vector<DirLight::DD>  dirLights;
    std::vector<PointLight::DD>  pointLights;
    std::pair<EnvMapLight::SP,affine3f> envLight;
    
    for (int i=0;i<instances.groups.size();i++) {
      Group *group = instances.groups[i].get();
      if (!group) continue;
      if (!group->lights) continue;
      for (auto &light : group->lights->items) {
        if (!light) continue;
        if (QuadLight::SP quadLight = light->as<QuadLight>()) {
          quadLights.push_back(quadLight->getDD(instances.xfms[i]));
          continue;
        } 
        if (DirLight::SP dirLight = light->as<DirLight>()) {
          dirLights.push_back(dirLight->getDD(instances.xfms[i]));
          continue;
        }
        if (PointLight::SP pointLight = light->as<PointLight>()) {
          pointLights.push_back(pointLight->getDD(instances.xfms[i]));
          continue;
        }
        if (EnvMapLight::SP el = light->as<EnvMapLight>()) {
          envLight = {el,instances.xfms[i]};
          continue;
        }
        throw std::runtime_error("un-handled type of light!?");
      }
    }
    world->set(envLight.first,envLight.second);
    world->set(quadLights);
    world->set(dirLights);
    world->set(pointLights);
  
    // ==================================================================
    // generate all (per device) instance lists. note each BGGroup can
    // contain more than one rtcGroup, so theres's not a one-to-one
    // between barney instance list transform array and rtc instance
    // list transform array
    // ==================================================================
    
    std::vector<int>          inputInstIDs;
    for (auto device : *devices) {
      PLD *pld = getPLD(device);
      std::vector<affine3f>     rtcTransforms;
      std::vector<rtc::Group *> rtcGroups;
      bool firstDevice = (device == (*devices)[0]);
      for (int i=0;i<instances.groups.size();i++) {
        Group *group = instances.groups[i].get();
        if (!group) continue;
        Group::PLD *groupPLD = group->getPLD(device);
      
        if (groupPLD->userGeomGroup) {
          rtcGroups.push_back(groupPLD->userGeomGroup);
          rtcTransforms.push_back(instances.xfms[i]);
          if (firstDevice)
            inputInstIDs.push_back(i);
          // if (!instances.userIDs.empty())
          //   userIDs.push_back(instances.userIDs[i]);
        }
        if (groupPLD->volumeGeomsGroup) {
          rtcGroups.push_back(groupPLD->volumeGeomsGroup);
          rtcTransforms.push_back(instances.xfms[i]);
          if (firstDevice)
            inputInstIDs.push_back(i);
        }

        if (groupPLD->triangleGeomGroup) {
          rtcGroups.push_back(groupPLD->triangleGeomGroup);
          rtcTransforms.push_back(instances.xfms[i]);
          if (firstDevice)
            inputInstIDs.push_back(i);
        }
        
        for (auto group : groupPLD->volumeGroups) {
          rtcGroups.push_back(group);
          rtcTransforms.push_back(instances.xfms[i]);
          if (firstDevice)
            inputInstIDs.push_back(i);
        }
      }
  
      if (pld->instanceGroup) {
        device->rtc->freeGroup(pld->instanceGroup);
        pld->instanceGroup = 0;
      }
      pld->instanceGroup
        = device->rtc->createInstanceGroup(rtcGroups,
                                           inputInstIDs,
                                           rtcTransforms);
      if (pld->instanceGroup)
        pld->instanceGroup->buildAccel();
    }
  }

}

  
