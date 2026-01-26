# VolumetricPlanets Plugin Architecture

This document describes how to structure VolumetricPlanets as an external plugin that extends Barney with Earth2 field types.

## Overview

The plugin system uses a registration mechanism where external field types can register themselves with Barney's `ScalarFieldRegistry` at static initialization time.

## Directory Structure

```
VolumetricPlanets/
├── CMakeLists.txt
├── devices/
│   └── barney/
│       ├── CMakeLists.txt
│       ├── fields/
│       │   ├── RegisterEarth2Fields.cpp  # Registration file
│       │   ├── PlanetField.h
│       │   ├── PlanetField.cu
│       │   ├── PlanetField.dev.cu
│       │   ├── CloudField.h
│       │   ├── CloudField.cu
│       │   ├── CloudField.dev.cu
│       │   ├── MagneticField.h
│       │   ├── MagneticField.cu
│       │   ├── MagneticField.dev.cu
│       │   ├── AuroraField.h
│       │   ├── AuroraField.cu
│       │   └── AuroraField.dev.cu
│       └── anari/
│           ├── SpatialFieldEarth2.cpp
│           └── SpatialFieldEarth2.h
└── README.md
```

## Top-Level CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.18)
project(VolumetricPlanets LANGUAGES CXX CUDA)

# Find or build Barney as a dependency
find_package(barney REQUIRED)
# OR: add_subdirectory(external/barney)

# Add the plugin
add_subdirectory(devices/barney)
```

## Plugin CMakeLists.txt

```cmake
# devices/barney/CMakeLists.txt

# Define Earth2 field sources
set(EARTH2_HOST_SOURCES
  fields/PlanetField.h
  fields/PlanetField.cu
  fields/CloudField.h
  fields/CloudField.cu
  fields/MagneticField.h
  fields/MagneticField.cu
  fields/AuroraField.h
  fields/AuroraField.cu
  fields/RegisterEarth2Fields.cpp  # <-- Registration happens here
  anari/SpatialFieldEarth2.cpp
  anari/SpatialFieldEarth2.h
)

set(EARTH2_DEVICE_SOURCES
  fields/PlanetField.dev.cu
  fields/CloudField.dev.cu
  fields/MagneticField.dev.cu
  fields/AuroraField.dev.cu
)

# Option 1: Compile as a separate library linked into barney
# This requires barney to expose the right CMake targets and headers

if (BARNEY_BACKEND_OPTIX)
  add_library(barney_earth2_optix STATIC ${EARTH2_HOST_SOURCES})
  
  # Compile device programs and embed PTX
  foreach(src ${EARTH2_DEVICE_SOURCES})
    get_filename_component(basename "${src}" NAME_WE)
    embed_ptx(
      OUTPUT_TARGET      barney-earth2-${basename}-ptx
      PTX_LINK_LIBRARIES barney_config_ptx barney_rtc_optix
      SOURCES            ${src}
    )
    target_link_libraries(barney_earth2_optix PRIVATE barney-earth2-${basename}-ptx)
  endforeach()
  
  target_link_libraries(barney_earth2_optix PUBLIC 
    barney::barney_optix
    barney::barney_config
  )
  
  set_library_properties(barney_earth2_optix)
  
  # Link the plugin into the main barney library
  # This requires barney to be built as part of the same project
  target_link_libraries(barney PRIVATE barney_earth2_optix)
  target_link_libraries(barney_static PRIVATE barney_earth2_optix)
endif()

# Similar for embree and cuda backends...
```

## Registration Code Example

The plugin needs a registration file to register all Earth2 field types. This happens at static initialization time, before `main()` runs.

```cpp
// VolumetricPlanets/devices/barney/fields/RegisterEarth2Fields.cpp
#include "barney/volume/ScalarFieldRegistry.h"
#include "PlanetField.h"
#include "CloudField.h"
#include "MagneticField.h"
#include "AuroraField.h"

namespace BARNEY_NS {

// Register all Earth2 field types
// These macros create static objects that register factories before main()
BARNEY_REGISTER_SCALAR_FIELD("planet", PlanetField)
BARNEY_REGISTER_SCALAR_FIELD("clouds", CloudField)
BARNEY_REGISTER_SCALAR_FIELD("magnetic", MagneticField)
BARNEY_REGISTER_SCALAR_FIELD("aurora", AuroraField)

} // namespace BARNEY_NS
```

That's it! No changes needed in individual field implementation files. The macro handles everything.

## How It Works

1. **Barney Core**: Provides only the registration infrastructure (`ScalarFieldRegistry`) and registers its own core types (structured, unstructured, NanoVDB, BlockStructuredAMR).

2. **Static Registration in Plugin**: The VolumetricPlanets plugin contains `RegisterEarth2Fields.cpp`, which uses the `BARNEY_REGISTER_SCALAR_FIELD` macro to register Earth2 field types at static initialization time (before `main()` runs).

3. **Factory Lookup**: When `ScalarField::create(context, devices, "planet")` is called:
   - Barney's built-in types are registered (one-time initialization)
   - The registry is queried for "planet"
   - If VolumetricPlanets was linked, "planet" factory exists and creates a `PlanetField`
   - If VolumetricPlanets was NOT linked, the factory doesn't exist and barney warns about unsupported type

4. **Clean Separation**: 
   - Barney core has NO knowledge of Earth2 fields
   - Earth2 fields live entirely in VolumetricPlanets
   - The registry acts as the integration point

## ANARI Interface

For ANARI support, you'll need to extend the ANARI wrapper. Two approaches:

### Approach A: Fork ANARI Wrapper (Simpler)

Keep `anari/SpatialField.cpp` in VolumetricPlanets with the Earth2 field types, and rebuild the ANARI library with your plugin.

### Approach B: Dynamic ANARI Extension (More Complex)

Create a mechanism where ANARI can discover parameter schemas from registered field types. This requires adding:

1. A virtual method to `ScalarField` for parameter description
2. ANARI query generation from the registry
3. Dynamic parameter handling in the ANARI wrapper

Approach A is recommended initially.

## Build Integration

To build both barney and VolumetricPlanets together:

```bash
# Structure
workspace/
├── barney/          # Your fork of barney
└── VolumetricPlanets/

# Build
cd VolumetricPlanets
mkdir build && cd build
cmake .. -Dbarney_DIR=../../barney/build
make -j
```

Or use CMake's `FetchContent` / `add_subdirectory` to build barney as part of VolumetricPlanets.

## Migration Steps

1. **Keep Core Enhancements in Barney**: Volume GI, `ScalarFieldRegistry`, bug fixes
2. **Move Earth2 Fields**: Copy Planet/Cloud/Magnetic/Aurora to VolumetricPlanets
3. **Update Build System**: Create plugin CMakeLists as shown above
4. **Test Registration**: Ensure fields register and create properly
5. **ANARI Integration**: Handle ANARI parameter discovery

## Benefits

- **Separation of Concerns**: Core barney vs. domain-specific Earth2 code
- **Independent Versioning**: VolumetricPlanets can evolve separately
- **Cleaner Upstream Contributions**: Core improvements can be submitted to nvidia/barney without Earth2-specific code
- **Reusable Pattern**: Other projects can create their own field type plugins

## Limitations

- **ANARI Query System**: Currently hardcoded - needs manual updating in plugin's ANARI wrapper
- **Build Complexity**: Requires coordinated builds between barney and plugin
- **Link-Time Registration**: Must link plugin library to get field types (can't load at runtime)
