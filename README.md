The module **libmodule_pd_preprocessor.so** is used to scale, cast and convert process data device members.

# Functional principle 

The posix timer module supports two different modes. 

## Modes

Two different modes are supported by **module_posix_timer**.

* ***nanosleep :*** In this mode the **module_posix_timer** main thread just does a nanosleep until the period time has been elapsed. If the nanosleep call was interrupted by some signal it will sleep until the calculated period end time has been reached. This mode is easy and efficient as well. The **module_posix_timer*** thread should run at a very high priority to ensure, that it will be waken up when it's necessary. 
* ***posix_timer :*** This mode creates a timer with timer_create. It configures the timer and connects it to the given signal number from the configuration string. 

## Example config file

This example config file can be used as a template for own configurations.

```yaml
# Configuration file for module_pd_preprocessor.
#
# vim: ft=yaml

#########################################################
# logging settings

# Standard robotkernel module local loglevel.
#loglevel: verbose

#########################################################
# direct device settings
devices:
  axis_0_msr:
    type: inputs
    pd_name: dsp402.axis_0.inputs.pd
    entries:
    - field_name: ecat.slave_1.inputs.pd.Position
      alias: position
      convert_to: double
      cast_to: int32_t
      scaling: 5.992112452678286e-06
    - field_name: ecat.slave_1.inputs.pd.Statusword
      alias: statusword
    - field_name: ecat.slave_1.inputs.pd.Modes of operation display
      alias: modes_of_operation_display
  axis_0_cmd:
    type: outputs
    pd_name: dsp402.axis_0.outputs.pd
    entries:
    - field_name: ecat.slave_1.outputs.pd.Controlword
      alias: controlword
    - field_name: ecat.slave_1.outputs.pd.Modes of operation
      alias: modes_of_operation
    - field_name: ecat.slave_1.outputs.pd.Target position
      alias: target_position
      convert_to: double
      cast_to: uint32_t
      scaling: 166886.05360752725
```

## Trigger device

This module registers a trigger device to the robotkernel with the following naming schemeː

```
<name>.posix_timer.trigger
```

## Process data device

The module also provides a cyclic process. It contains the actual timer interval.

```
<name>.inputs.pd
```
