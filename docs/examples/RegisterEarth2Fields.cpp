// Example file that would be in VolumetricPlanets plugin
// Path: VolumetricPlanets/devices/barney/fields/RegisterEarth2Fields.cpp

#include "barney/volume/ScalarFieldRegistry.h"
#include "barney/volume/PlanetField.h"
#include "barney/volume/CloudField.h"
#include "barney/volume/MagneticField.h"
#include "barney/volume/AuroraField.h"

namespace BARNEY_NS {

// Register all Earth2 field types when this compilation unit is linked
// These static objects will execute before main() and register the factories

BARNEY_REGISTER_SCALAR_FIELD("planet", PlanetField)
BARNEY_REGISTER_SCALAR_FIELD("clouds", CloudField)
BARNEY_REGISTER_SCALAR_FIELD("magnetic", MagneticField)
BARNEY_REGISTER_SCALAR_FIELD("aurora", AuroraField)

} // namespace BARNEY_NS
