# modality-gazebo

## Example

1. Install dependencies.
  ```
  sudo apt install libignition-gazebo7-dev libgz-cmake3-dev libignition-plugin-dev modality-sdk
  ```
2. Add `<auth_token>YOUR_AUTH_TOKEN_HEX_HERE</auth_token>` to [examples/world.sdf](examples/world.sdf) or set `MODALITY_AUTH_TOKEN` env var.
3. Build the plugin:
  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```
4. Run the example world:
  ```bash
  make run-example
  ```
