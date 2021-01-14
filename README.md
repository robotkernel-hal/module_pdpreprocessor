The module **libmodule_pd_preprocessor.so** is used to scale, cast and convert process data device members.

# Functional principle 

The pd preprocessor module supports different modes. 

## Scaling

Values from process data can be scaled by a given floating point value.

## Casting

Target values may be cast to a different data type.

## Conversion

Target values can be converted to an other data type prior to be scaled.

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
<name>.inputs.trigger
```

## Process data device

The module also provides a cyclic process. 

```
<name>.inputs.pd
```
