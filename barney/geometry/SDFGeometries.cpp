// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "barney/geometry/SDFGeometries.h"
#include "barney/Context.h"
#include "barney/ModelSlot.h"
#include <algorithm>

namespace BARNEY_NS {

  RTC_IMPORT_USER_GEOM(SDFGeometries, SDFGeometries, SDFGeometries::DD, false, true);

  SDFGeometries::SDFGeometries(Context *context, DevGroup::SP devices)
    : Geometry(context, devices)
  {}

  void SDFGeometries::commit()
  {
    if (!sdf || sdf->count == 0) {
      std::cout << "#bn.sdf: warning - empty or missing primitive.sdf array"
                << std::endl;
      return;
    }

    const size_t numPrimitives = sdf->count / sizeof(SDFPrimitive);
    if (numPrimitives == 0 || (sdf->count % sizeof(SDFPrimitive)) != 0) {
      std::cout << "#bn.sdf: primitive.sdf size must be a multiple of "
                << sizeof(SDFPrimitive) << " bytes" << std::endl;
      return;
    }

    for (auto device : *devices) {
      PLD *pld = getPLD(device);
      if (pld->userGeoms.empty()) {
        rtc::GeomType *gt =
          device->geomTypes.get(createGeomType_SDFGeometries);
        pld->userGeoms = {gt->createGeom()};
      }
      rtc::Geom *geom = pld->userGeoms[0];
      geom->setPrimCount((int)numPrimitives);

      DD dd;
      Geometry::writeDD(dd, device);
      dd.geometries = (const SDFPrimitive *)(sdf ? sdf->getDD(device) : nullptr);
      dd.neighbours = (const uint64_t *)(neighbour ? neighbour->getDD(device)
                                                    : nullptr);
      dd.numGeometries = (uint32_t)numPrimitives;
      dd.epsilon = epsilon;
      dd.nbMarchIterations = nbMarchIterations;
      dd.blendFactor = blendFactor;
      dd.blendLerpFactor = blendLerpFactor;
      dd.omega = omega;
      dd.distanceFromCamera = distanceFromCamera;
      dd.noiseFactor = noiseFactor;
      dd.blendDistanceFromCamera = blendDistanceFromCamera;
      geom->setDD(&dd);
    }
  }

  bool SDFGeometries::setData(const std::string &member,
                              const barney_api::Data::SP &value)
  {
    if (Geometry::setData(member, value))
      return true;
    if (member == "primitive.sdf") {
      sdf = value ? value->as<PODData>() : PODData::SP();
      return true;
    }
    if (member == "primitive.neighbor" || member == "primitive.neighbour") {
      neighbour = value ? value->as<PODData>() : PODData::SP();
      return true;
    }
    return false;
  }

  bool SDFGeometries::set1f(const std::string &member, const float &value)
  {
    if (Geometry::set1f(member, value))
      return true;
    if (member == "epsilon") {
      epsilon = value;
      return true;
    }
    if (member == "blendFactor") {
      blendFactor = value;
      return true;
    }
    if (member == "blendLerpFactor") {
      blendLerpFactor = value;
      return true;
    }
    if (member == "omega") {
      omega = std::clamp(value, 0.f, 1.f);
      return true;
    }
    if (member == "distanceFromCamera") {
      distanceFromCamera = value;
      return true;
    }
    if (member == "noiseFactor") {
      noiseFactor = std::clamp(value, 0.f, 1.f);
      return true;
    }
    if (member == "blendDistanceFromCamera") {
      blendDistanceFromCamera = value;
      return true;
    }
    return false;
  }

  bool SDFGeometries::set1i(const std::string &member, const int &value)
  {
    if (Geometry::set1i(member, value))
      return true;
    if (member == "nbMarchIterations") {
      nbMarchIterations = (uint32_t)std::max(value, 1);
      return true;
    }
    return false;
  }

} // namespace BARNEY_NS
