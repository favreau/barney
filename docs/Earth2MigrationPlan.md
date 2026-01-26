# Migration Plan: Separating Earth2 Fields from Barney Core

## What STAYS in Barney (Core Infrastructure)

### 1. Registration System
- `barney/volume/ScalarFieldRegistry.h` - NEW: Plugin registration infrastructure
- `barney/volume/ScalarField.h` - Existing base class
- `barney/volume/ScalarField.cpp` - Modified to use registry for built-in types only

### 2. Volume GI Support (Generic Feature)
- `barney/volume/Volume.h` - Added `enableGI` member
- `barney/volume/Volume.cpp` - Added `enableGI` parameter handling
- `barney/render/Ray.h` - Added `enableGI` to `setVolumeHit()`
- `barney/packedBSDF/Phase.h` - Added GI flag storage
- `barney/kernels/shadeRays.cu` - GI scatter logic
- `barney/volume/MCAccelerator.h` - Passes GI flag to ray

### 3. Bug Fixes
- `anari/barney_math.h` - Fixed `bnSet4fc` bug (z,z → z,w)

### 4. Core Volume Types (Built-in)
- `barney/volume/StructuredData.*`
- `barney/volume/NanoVDB.*`
- `barney/umesh/common/UMeshField.*`
- `barney/amr/BlockStructuredField.*`

## What MOVES to VolumetricPlanets Plugin

### 1. Earth2 Field Implementations
FROM `barney/volume/` TO `VolumetricPlanets/devices/barney/fields/`:
- `PlanetField.h`
- `PlanetField.cu`
- `PlanetField.dev.cu`
- `CloudField.h`
- `CloudField.cu`
- `CloudField.dev.cu`
- `MagneticField.h`
- `MagneticField.cu`
- `MagneticField.dev.cu`
- `AuroraField.h`
- `AuroraField.cu`
- `AuroraField.dev.cu`

### 2. Registration File (NEW in VolumetricPlanets)
- `VolumetricPlanets/devices/barney/fields/RegisterEarth2Fields.cpp`
  - Uses `BARNEY_REGISTER_SCALAR_FIELD` macro to register all 4 field types

### 3. ANARI Earth2 Wrappers
FROM `anari/SpatialField.cpp` TO `VolumetricPlanets/devices/barney/anari/`:
- Extract Earth2-specific classes from `anari/SpatialField.cpp`:
  - `PlanetSpatialField`
  - `CloudSpatialField`
  - `MagneticFieldSpatialField`
  - `AuroraSpatialField`
- Move to new file: `VolumetricPlanets/devices/barney/anari/SpatialFieldEarth2.cpp`

### 4. ANARI Query Extensions
FROM `anari/generated/anari_library_barney_queries.cpp`:
- Extract Earth2 parameter info functions
- Move to: `VolumetricPlanets/devices/barney/anari/anari_library_earth2_queries.cpp`

## What Gets MODIFIED in Barney

### anari/SpatialField.cpp
**REMOVE** Earth2-specific code:
```cpp
// Remove these includes:
#include "barney/volume/PlanetField.h"
#include "barney/volume/CloudField.h"
#include "barney/volume/MagneticField.h"
#include "barney/volume/AuroraField.h"

// Remove these classes entirely:
struct PlanetSpatialField : public SpatialField { ... };
struct CloudSpatialField : public SpatialField { ... };
struct MagneticFieldSpatialField : public SpatialField { ... };
struct AuroraSpatialField : public SpatialField { ... };

// Remove from SpatialField::createBarneyScalarField():
if (subtype == "planet") return new PlanetSpatialField(...);
if (subtype == "clouds") return new CloudSpatialField(...);
// etc.
```

### barney/CMakeLists.txt
**REMOVE** Earth2 sources from `HOST_SOURCES`:
```cmake
# Remove:
volume/PlanetField.h
volume/PlanetField.cu
volume/CloudField.h
volume/CloudField.cu
volume/MagneticField.h
volume/MagneticField.cu
volume/AuroraField.h
volume/AuroraField.cu
```

**REMOVE** from `DEVICE_PROGRAM_SOURCES`:
```cmake
# Remove:
volume/PlanetField.dev.cu
volume/CloudField.dev.cu
volume/MagneticField.dev.cu
volume/AuroraField.dev.cu
```

## VolumetricPlanets CMakeLists Structure

```cmake
# VolumetricPlanets/devices/barney/CMakeLists.txt

set(EARTH2_SOURCES
  fields/RegisterEarth2Fields.cpp  # <-- Key registration file
  fields/PlanetField.cu
  fields/CloudField.cu
  fields/MagneticField.cu
  fields/AuroraField.cu
  anari/SpatialFieldEarth2.cpp
)

set(EARTH2_DEVICE_SOURCES
  fields/PlanetField.dev.cu
  fields/CloudField.dev.cu
  fields/MagneticField.dev.cu
  fields/AuroraField.dev.cu
)

# Link into barney's optix backend
add_library(barney_earth2_optix STATIC ${EARTH2_SOURCES})
target_link_libraries(barney_earth2_optix PUBLIC barney::barney_optix)

# Embed PTX for device programs
foreach(src ${EARTH2_DEVICE_SOURCES})
  # ... embed_ptx() ...
endforeach()

# Link plugin into barney
target_link_libraries(barney PRIVATE barney_earth2_optix)
```

## Benefits of This Architecture

1. **Clean Separation**: Barney core contains only generic rendering infrastructure
2. **Upstream Contributions**: Volume GI and bug fixes can be contributed to nvidia/barney without Earth2 code
3. **Independent Development**: VolumetricPlanets evolves on its own schedule
4. **Reusable Pattern**: Other projects can create field type plugins using the same mechanism
5. **No Runtime Overhead**: Registration happens at static init time, zero runtime cost

## Migration Checklist

- [ ] Add `ScalarFieldRegistry.h` to barney
- [ ] Modify `ScalarField.cpp` to use registry (built-in types only)
- [ ] Add Volume GI support (already done in feature/planet-fields)
- [ ] Create VolumetricPlanets repo structure
- [ ] Move Earth2 field `.h/.cu/.dev.cu` files to VolumetricPlanets
- [ ] Create `RegisterEarth2Fields.cpp` in VolumetricPlanets
- [ ] Extract Earth2 ANARI classes from barney to VolumetricPlanets
- [ ] Update CMakeLists in both repos
- [ ] Test build: barney standalone (should work without Earth2)
- [ ] Test build: VolumetricPlanets + barney (should include Earth2 fields)
- [ ] Verify field types register and render correctly
- [ ] Update documentation

## Current Branch State

The `feature/planet-fields` branch currently has Earth2 fields IN barney. This is fine for now - the migration to VolumetricPlanets can happen later. The key work done now is:

1. ✅ Added registration infrastructure to barney
2. ✅ Prepared barney to support external plugins
3. ✅ Documented the plugin architecture

Next step: Actually perform the migration when you're ready to create the VolumetricPlanets repo.
