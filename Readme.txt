Battery Charge/Discharge and Heater Controller for small electric vehicles.

This system provides Battery telemetry services such as Voltage, Current, and Temperature. It also regulates charge/discharge
voltages, limits current when battery temps are too high or too low, controls a battery heater for cold environments, and basic
cooling fan control. Works with lithium ion batteries, but does not provide cell ballancing. That must be done with a
dedicated BMS. The circuit provided should handle up to 30 amps with proper cooling. It also does basic error logging and will
shut down the system on events such as under/over voltage, over current, temps out of range, internal CPU errors, etc. with room to expand.

The firmware has yet to be fully updated to work with this schematic, though, it should mostly work at this time. Jun, 2021