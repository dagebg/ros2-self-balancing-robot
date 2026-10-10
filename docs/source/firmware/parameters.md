# Robot parameters

Physical and timing constants live in one file, `config/robot_params.yaml`, and every consumer
reads them from there (see {doc}`../development/adr/0004-single-source-robot-parameters`):

- the firmware, through the generated header `include/robot_config.hpp`;
- the ROS 2 packages and the simulation, which load the YAML directly.

After editing the YAML, regenerate the header:

```bash
python tools/gen_config.py
```

pre-commit and CI fail if the header is out of date. Values marked **placeholder** have not been
measured on the real robot yet; the firmware logs a warning at boot while any placeholder remains.

## Current values

This table is generated from `config/robot_params.yaml` when the docs are built.

```{include} ../_generated/robot_params_table.md
```
