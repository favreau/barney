# Scalar Field Plugin System

This branch adds a registration system that allows external projects to extend Barney with custom scalar field types.

## What Changed

### New Files
- `barney/volume/ScalarFieldRegistry.h` - Plugin registration infrastructure

### Modified Files
- `barney/volume/ScalarField.cpp` - Refactored to use registry instead of hard-coded if/else chain

## How It Works

### For Barney Core
Built-in types (structured, unstructured, BlockStructuredAMR, NanoVDB) are registered automatically when `ScalarField::create()` is first called.

### For External Plugins

External projects can register custom field types that will be discovered at runtime:

```cpp
// In your plugin's .cpp file
#include "barney/volume/ScalarFieldRegistry.h"
#include "MyCustomField.h"

namespace BARNEY_NS {

// Register your custom field type
BARNEY_REGISTER_SCALAR_FIELD("myfield", MyCustomField)

} // namespace BARNEY_NS
```

When your plugin is linked into an application that uses Barney, the registration happens automatically at static initialization time (before `main()` runs).

## Benefits

1. **Extensibility**: External projects can add field types without modifying Barney core
2. **Clean Separation**: Domain-specific field types can live in separate repositories
3. **Upstream Contributions**: Core improvements can be contributed to Barney without plugin-specific code
4. **Zero Runtime Cost**: Registration happens once at startup, no overhead during rendering

## Example Use Case

The VolumetricPlanets project uses this system to add Earth2-specific field types (planet, clouds, magnetic, aurora) without modifying Barney's source code.

## API Reference

### ScalarFieldRegistry

```cpp
class ScalarFieldRegistry {
  static ScalarFieldRegistry& instance();
  
  void registerType(const std::string& type, ScalarFieldFactory factory);
  ScalarField::SP create(Context* ctx, const DevGroup::SP& devs, const std::string& type);
  bool hasType(const std::string& type) const;
};
```

### BARNEY_REGISTER_SCALAR_FIELD Macro

```cpp
BARNEY_REGISTER_SCALAR_FIELD(TYPE_NAME, CLASS_NAME)
```

Creates a static registrar object that registers `CLASS_NAME` with the given `TYPE_NAME` string.

**Parameters:**
- `TYPE_NAME` - String literal for the field type (e.g., "myfield")
- `CLASS_NAME` - C++ class name that inherits from `ScalarField` (e.g., MyCustomField)

**Requirements:**
- `CLASS_NAME` must have a constructor: `CLASS_NAME(Context*, const DevGroup::SP&)`
- Must be used in `BARNEY_NS` namespace

## Thread Safety

The registry is thread-safe and can be called from multiple threads simultaneously.
