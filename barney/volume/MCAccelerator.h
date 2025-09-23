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
#include "barney/DeviceGroup.h"
#include "barney/volume/MCGrid.h"
#include "barney/volume/Volume.h"
#include "barney/geometry/IsoSurface.h"
#include "barney/volume/DDA.h"
#include "barney/render/World.h"
#include "barney/render/OptixGlobals.h"
#include "barney/material/DeviceMaterial.h"
#if RTC_DEVICE_CODE
# include "rtcore/TraceInterface.h"
#endif

namespace BARNEY_NS {
  using render::Ray;
  using render::DeviceMaterial;
 
  template<typename SFSampler>
  struct MCVolumeAccel : public VolumeAccel 
  {
    struct DD 
    {
      Volume::DD<SFSampler> volume;
      MajorantsGrid::DD     mcGrid;
    };

    struct PLD {
      rtc::Geom  *geom  = 0;
      rtc::Group *group = 0;
    };
    PLD *getPLD(Device *device) 
    { return &perLogical[device->contextRank()]; } 
    std::vector<PLD> perLogical;

    DD getDD(Device *device)
    {
      DD dd;
      dd.volume = volume->getDD(device,sfSampler);
      dd.mcGrid = majorantsGrid->getDD(device);
      return dd;
    }

    MCVolumeAccel(Volume *volume,
                  GeomTypeCreationFct creatorFct,
                  const std::shared_ptr<SFSampler> &sfSampler);

      GeomTypeCreationFct const creatorFct;
    
    void build(bool full_rebuild) override;

#if BARNEY_DEVICE_PROGRAM
    /*! optix bounds prog for this class of accels */
    static inline __rtc_device
    void boundsProg(const rtc::TraceInterface &ti,
                    const void *geomData,
                    owl::common::box3f &bounds,
                    const int32_t primID);
    /*! optix isec prog for this class of accels */
    static inline __rtc_device
    void isProg(rtc::TraceInterface &ti);
#endif 
    
    MajorantsGrid::SP majorantsGrid;
    const std::shared_ptr<SFSampler> sfSampler;
  };
  
  template<typename SFSampler>
  struct MCIsoSurfaceAccel : public IsoSurfaceAccel 
  {
    struct DD 
    {
      IsoSurface::DD<SFSampler> isoSurface;
      MCGrid::DD mcGrid;
    };
    
    struct PLD {
      rtc::Geom  *geom  = 0;
      rtc::Group *group = 0;
    };
    PLD *getPLD(Device *device) 
    { return &perLogical[device->contextRank()]; } 
    std::vector<PLD> perLogical;
    
    DD getDD(Device *device)
    {
      DD dd;
      dd.isoSurface = isoSurface->getDD(device,sfSampler);
      // dd.sf     = sfSampler->getDD(device);
      dd.mcGrid = mcGrid->getDD(device);
      return dd;
    }
    
    MCIsoSurfaceAccel(IsoSurface *isoSurface,
                      GeomTypeCreationFct creatorFct,
                      const std::shared_ptr<SFSampler> &sfSampler);
    
    GeomTypeCreationFct const creatorFct;
    
    void build() override;
    
#if BARNEY_DEVICE_PROGRAM
    /*! optix bounds prog for this class of accels */
    static inline __rtc_device
    void boundsProg(const rtc::TraceInterface &ti,
                    const void *geomData,
                    owl::common::box3f &bounds,
                    const int32_t primID);
    /*! optix isec prog for this class of accels */
    static inline __rtc_device
    void isProg(rtc::TraceInterface &ti);
#endif 
    
    MCGrid::SP       mcGrid;
    const std::shared_ptr<SFSampler> sfSampler;
  };
  
  // ==================================================================
  // INLINE IMPLEMENTATION SECTION
  // ==================================================================
  
  template<typename SFSampler>
  void MCVolumeAccel<SFSampler>::build(bool full_rebuild) 
  {
    if (!majorantsGrid) {
      auto mcGrid = volume->sf->getMCs();
      majorantsGrid = std::make_shared<MajorantsGrid>(mcGrid);
    }
    majorantsGrid->computeMajorants(&volume->xf);
    sfSampler->build();
    
    for (auto device : *devices) {
      SetActiveGPU forDuration(device);
      
      // build our own internal per-device data: one geom, and one
      // group that contains it.
      PLD *pld = getPLD(device);
      if (!pld->geom) {
        rtc::GeomType *gt
          = device->geomTypes.get(creatorFct);
        // build a single-prim geometry, that single prim is our
        // entire MC/DDA grid
        pld->geom = gt->createGeom();
        pld->geom->setPrimCount(1);
      }
      rtc::Geom *geom = pld->geom;
      DD dd = getDD(device);
      geom->setDD(&dd);

      if (!pld->group) {
        // now put that into a instantiable group, and build it.
        pld->group = device->rtc->createUserGeomsGroup({geom});
      }
      pld->group->buildAccel();

      // now let the actual volume we're building know about the
      // group we just created
      Volume::PLD *volumePLD = volume->getPLD(device);
      if (volumePLD->generatedGroups.empty()) 
        volumePLD->generatedGroups = { pld->group };
    }
  }



  template<typename SFSampler>
  void MCIsoSurfaceAccel<SFSampler>::build() 
  {
    mcGrid = isoSurface->sf->getMCs();
    sfSampler->build();
    PING;
    
    for (auto device : *devices) {
      SetActiveGPU forDuration(device);
      
      // build our own internal per-device data: one geom, and one
      // group that contains it.
      PLD *pld = getPLD(device);
      if (!pld->geom) {
        rtc::GeomType *gt
          = device->geomTypes.get(creatorFct);
        // build a single-prim geometry, that single prim is our
        // entire MC/DDA grid
        pld->geom = gt->createGeom();
        pld->geom->setPrimCount(1);
      }
      rtc::Geom *geom = pld->geom;
      DD dd = getDD(device);
      geom->setDD(&dd);
      
      IsoSurface::PLD *isoSurfacePLD = isoSurface->getPLD(device);
      isoSurfacePLD->userGeoms = { geom };
      // if (!pld->group) {
      //   // now put that into a instantiable group, and build it.
      //   pld->group = device->rtc->createUserGeomsGroup({geom});
      // }
      // pld->group->buildAccel();

      // // now let the actual volume we're building know about the
      // // group we just created
      // IsoSurface::PLD *isoSurfacePLD = isoSurface->getPLD(device);
      // if (isoSurfacePLD->generatedGroups.empty()) 
      //   isoSurfacePLD->generatedGroups = { pld->group };
    }
  }
  
  

  template<typename SFSampler>
  MCVolumeAccel<SFSampler>::
  MCVolumeAccel(Volume *volume,
                GeomTypeCreationFct creatorFct,
                const std::shared_ptr<SFSampler> &sfSampler)
    : VolumeAccel(volume),
      sfSampler(sfSampler),
      creatorFct(creatorFct)
  {
    perLogical.resize(devices->numLogical);
  }

  template<typename SFSampler>
  MCIsoSurfaceAccel<SFSampler>::
  MCIsoSurfaceAccel(IsoSurface *isoSurface,
                    GeomTypeCreationFct creatorFct,
                    const std::shared_ptr<SFSampler> &sfSampler)
    : IsoSurfaceAccel(isoSurface),
      sfSampler(sfSampler),
      creatorFct(creatorFct)
  {
    perLogical.resize(devices->numLogical);
  }
  
  // ------------------------------------------------------------------
  // device progs: macro-cell accel with DDA traversal
  // ------------------------------------------------------------------

#if BARNEY_DEVICE_PROGRAM && RTC_DEVICE_CODE
  template<typename SFSampler>
  inline __rtc_device
  void MCVolumeAccel<SFSampler>::boundsProg(const rtc::TraceInterface &ti,
                                            const void *geomData,
                                            owl::common::box3f &bounds,
                                            const int32_t primID)
  {
    const DD &self = *(DD*)geomData;
    bounds = self.volume.sfCommon.worldBounds;
  }
  
  template<typename SFSampler>
  inline __rtc_device
  void MCIsoSurfaceAccel<SFSampler>::boundsProg(const rtc::TraceInterface &ti,
                                                const void *geomData,
                                                owl::common::box3f &bounds,
                                                const int32_t primID)
  {
    const DD &self = *(DD*)geomData;
    bounds = self.isoSurface.sfCommon.worldBounds;
  }
  
  template<typename SFSampler>
  inline __rtc_device
  void MCIsoSurfaceAccel<SFSampler>::isProg(rtc::TraceInterface &ti)
  {
    const void *pd = ti.getProgramData();
           
    const DD &self = *(typename MCIsoSurfaceAccel<SFSampler>::DD*)pd;
    const render::World::DD &world = render::OptixGlobals::get(ti).world;
    // ray in world space
    Ray &ray = *(Ray*)ti.getPRD();
#ifdef NDEBUG
    const bool dbg = false;
#else
    const bool dbg = ray.dbg();
#endif
    
    box3f bounds = self.isoSurface.sfCommon.worldBounds;
    range1f tRange = { ti.getRayTmin(), ti.getRayTmax() };
    if (dbg) printf(" TRANGE BEFORE BOX %f %f\n",tRange.lower,tRange.upper);
    
    // ray in object space
    vec3f obj_org = ti.getObjectRayOrigin();
    vec3f obj_dir = ti.getObjectRayDirection();

    if (dbg) {
      printf("MCIsoAccel isec %f %f %f mcgrid %i %i %i\n",
             obj_dir.x,
             obj_dir.y,
             obj_dir.z,
             self.mcGrid.dims.x,
             self.mcGrid.dims.y,
             self.mcGrid.dims.z
             );
    }
    
    auto objRay = ray;
    objRay.org = obj_org;
    objRay.dir = obj_dir;

    if (!boxTest(objRay,tRange,bounds))
      return;

    if (dbg) printf(" TRANGE AFTER BOX %f %f\n",tRange.lower,tRange.upper);
    
    // ------------------------------------------------------------------
    // compute ray in macro cell grid space 
    // ------------------------------------------------------------------
    vec3f mcGridOrigin  = self.mcGrid.gridOrigin;
    vec3f mcGridSpacing = self.mcGrid.gridSpacing;

    vec3f dda_org = obj_org;
    vec3f dda_dir = obj_dir;

    dda_org = (dda_org - mcGridOrigin) * rcp(mcGridSpacing);
    dda_dir = dda_dir * rcp(mcGridSpacing);

#if 1
    Random rng(ray.rngSeed,hash(ti.getRTCInstanceIndex(),
                                ti.getGeometryIndex(),
                                ti.getPrimitiveIndex()));
#else
    Random rng(ray.rngSeed.next(hash(ti.getRTCInstanceIndex(),
                                     ti.getGeometryIndex(),
                                     ti.getPrimitiveIndex())));
#endif
    
    float tHit = ray.tMax;
    dda::dda3(dda_org,dda_dir,tRange.upper,
              vec3ui(self.mcGrid.dims),
              [&](const vec3i &cellIdx, float t0, float t1) -> bool
              {
                float _t0 = t0;
                float _t1 = t1;
                range1f tRange = range1f {t0,min(t1,ray.tMax)};
                if (tRange.lower >= tRange.upper) return true;
                
                range1f valueRange = self.mcGrid.scalarRange(cellIdx);

                if (dbg) printf("dda %i %i %i [%f %f] -> [%f %f]\n",
                                cellIdx.x,
                                cellIdx.y,
                                cellIdx.z,
                                tRange.lower,
                                tRange.upper,
                                valueRange.lower,
                                valueRange.upper);
                auto overlaps = [&](float isoValue)
                {
                  if (isnan(isoValue)) return false;
                  if (isoValue < valueRange.lower || isoValue > valueRange.upper)
                    return false;
                  return true;
                };
                auto overlaps_any = [&]()
                {
                  if (overlaps(self.isoSurface.isoValue)) return true;
                  return false;
                };
                auto intersect = [&](float isoValue)
                {
                  if (isnan(isoValue)) return;
                  if (isoValue < valueRange.lower || isoValue > valueRange.upper)
                    return;

                  float t
                    = (isoValue - valueRange.lower)
                    / (valueRange.upper-valueRange.lower);
                  t = lerp_l(t,tRange.lower,tRange.upper);
                  tHit = min(tHit,t);
                };
                auto intersect_all = [&]()
                {
                  intersect(self.isoSurface.isoValue);
                };
                
                if (!overlaps_any()) return true;

                float tt1 = t0;
                vec3f P = obj_org + tt1 * obj_dir;
                float ff1 = self.isoSurface.sfSampler.sample(P,dbg);
                int numSteps = 10; 
                for (int i=1;i<=numSteps;i++) {
                  float tt0 = tt1;
                  float ff0 = ff1;
                  tt1 = lerp_l(i/float(numSteps),_t0,_t1);
                  P = obj_org + tt1 * obj_dir;
                  ff1 = self.isoSurface.sfSampler.sample(P,dbg);
                  
                  valueRange.lower = ff0;
                  valueRange.upper = ff1;
                  // if (dbg)
                  //   printf("i %i [%f %f] -> t's %f %f\n",i,_t0,_t1,tt0,tt1);
                  tRange = range1f{tt0,tt1};

                  if (isnan(ff0+ff1)) continue;
                  
                  if (dbg)
                    printf(" ... t [%f %f] v [ %f %f ]\n",
                           tRange.lower,
                           tRange.upper,
                           valueRange.lower,
                           valueRange.upper);
                  if (overlaps_any()) {
                    intersect_all();
                    if (tHit < ray.tMax) {
                      return false;
                    }
                  }
                }

                return true;
              },
              /*NO debug:*/false
              );
    if (tHit >= ray.tMax) return;
    
    // ------------------------------------------------------------------
    // get texture coordinates
    // ------------------------------------------------------------------
    const vec3f osP  = obj_org + tHit * obj_dir;
    vec3f P  = ti.transformPointFromObjectToWorldSpace(osP);
    vec3f osN = - normalize(obj_dir);
    vec3f n   = - normalize(obj_dir);
    int primID    = ti.getPrimitiveIndex();
    int instID    = ti.getInstanceID();
                
    render::HitAttributes hitData;
    hitData.worldPosition   = P;
    hitData.worldNormal     = n;
    hitData.objectPosition  = osP;
    hitData.objectNormal    = osN;
    hitData.primID          = primID;
    hitData.instID          = instID;
    hitData.t               = tHit;
    hitData.isShadowRay     = ray.isShadowRay;
    float u = 0.f;
    float v = 0.f;
    auto interpolator
      = [u,v,dbg](const GeometryAttribute::DD &attrib) -> vec4f
      {
        return vec4f(1.f);
      };
    self.isoSurface.setHitAttributes(hitData,interpolator,world,dbg);

    // if (dbg) printf("matid %i, world mat %lx\n",self.materialID,world.materials); 
    const DeviceMaterial &material
      = world.materials[self.isoSurface.materialID];
      
    PackedBSDF bsdf
      = material.createBSDF(hitData,world.samplers,dbg);
    float opacity
      = bsdf.getOpacity(ray.isShadowRay,ray.isInMedium,
                        ray.dir,hitData.worldNormal,ray.dbg());
    // opacity = .85f;
    if (opacity < 1.f) {
      // ray.rngSeed.next((const uint32_t&)osP.x);
      // ray.rngSeed.next((const uint32_t&)osP.y);
      // ray.rngSeed.next((const uint32_t&)osP.z);
      // Random rng(ray.rngSeed,290374u);
      if (rng() > opacity) {
        // ti.ignoreIntersection();
        return;
      }
    }
    material.setHit(ray,hitData,world.samplers,dbg);
  }
  
  template<typename SFSampler>
  inline __rtc_device
  void MCVolumeAccel<SFSampler>::isProg(rtc::TraceInterface &ti)
  {
    const void *pd = ti.getProgramData();
           
    const DD &self = *(typename MCVolumeAccel<SFSampler>::DD*)pd;
    const render::World::DD &world = render::OptixGlobals::get(ti).world;
    // ray in world space
    Ray &ray = *(Ray*)ti.getPRD();
#ifdef NDEBUG
    enum { dbg = false };
#else
    const bool dbg = ray.dbg();
#endif
    
    box3f bounds = self.volume.sfCommon.worldBounds;
    range1f tRange = { ti.getRayTmin(), ti.getRayTmax() };
    
    // ray in object space
    vec3f obj_org = ti.getObjectRayOrigin();
    vec3f obj_dir = ti.getObjectRayDirection();

    if (dbg) {
      printf("MCVolumeAccel isec %f %f %f\n",
             obj_dir.x,
             obj_dir.y,
             obj_dir.z
             );
    }
    
    auto objRay = ray;
    objRay.org = obj_org;
    objRay.dir = obj_dir;

    if (!boxTest(objRay,tRange,bounds))
      return;
    
    // ------------------------------------------------------------------
    // compute ray in macro cell grid space 
    // ------------------------------------------------------------------
    vec3f mcGridOrigin  = self.mcGrid.gridOrigin;
    vec3f mcGridSpacing = self.mcGrid.gridSpacing;

    vec3f dda_org = obj_org;
    vec3f dda_dir = obj_dir;

    dda_org = (dda_org - mcGridOrigin) * rcp(mcGridSpacing);
    dda_dir = dda_dir * rcp(mcGridSpacing);

    Random rng(ray.rngSeed,hash(ti.getRTCInstanceIndex(),
                                ti.getGeometryIndex(),0));
    // Random rng(ray.rngSeed,hash(ti.getRTCInstanceIndex(),
    //                             ti.getGeometryIndex(),
    //                             ti.getPrimitiveIndex()));
    dda::dda3(dda_org,dda_dir,tRange.upper,
              vec3ui(self.mcGrid.dims),
              [&](const vec3i &cellIdx, float t0, float t1) -> bool
              {
                const float majorant = self.mcGrid.majorant(cellIdx);
                
                if (majorant == 0.f) return true;
                
                vec4f   sample = 0.f;
                range1f tRange = {t0,min(t1,ray.tMax)};
                if (!Woodcock::sampleRange(sample,
                                           self.volume,
                                           obj_org,
                                           obj_dir,
                                           tRange,
                                           majorant,
                                           rng,
                                           dbg)) 
                  return true;
                if (dbg) printf("woodcock hit sample %f %f %f:%f\n",
                                sample.x,
                                sample.y,
                                sample.z,
                                sample.w);
                
                vec3f P_obj = obj_org + tRange.upper * obj_dir;
                vec3f P = ti.transformPointFromObjectToWorldSpace(P_obj);
                ray.setVolumeHit(P,
                                 tRange.upper,
                                 getPos(sample));
                ti.reportIntersection(tRange.upper, 0);
                return false;
              },
              /*NO debug:*/false
              );
  }
#endif
}
