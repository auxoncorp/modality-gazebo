# modality-gazebo

## Example

1. Add `<auth_token>YOUR_AUTH_TOKEN_HEX_HERE</auth_token>` to [examples/world.sdf](examples/world.sdf) or set `MODALITY_AUTH_TOKEN` env var.
2. Build the plugin:
  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```
3. Run the example world:
  ```bash
  make run-example
  ```
