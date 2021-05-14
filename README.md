# openbmc-gxp-dbus-sensors

openbmc-dbus-gxp-sensors is a component of HPE's OpenBMC port for ProLiant server architectures using the GXP BMC ASIC.  It is a collection of sensor applications similar to upstream OpenBMC dbus-sensors, but for HPE GXP-based devices (e.g. coretemp, CPLD fan,..etc.).

## Entity Manager Interoperability
OpenBMC's Entity Manager creates dbus objects at `/xyz/openbmc_project/inventory` path with `xyz.openbmc_project.Configuration.*` interface according to the matched json file under `/usr/share/entity-manager/configuration/*`

gxp-dbus-sensors scans the dbus object which match `xyz.openbmc_project.Configuration.*` and creates a dbus objects at `/xyz/openbmc_project/sensors/temperature/*` path with the `xyz.openbmc_project.Sensor.Value` interface so that `gxp-dbus-sensor` can export the sensor status to this dbus object's `xyz.openbmc_project.Sensor.Value` interface as its property value.
