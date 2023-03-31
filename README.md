# modality-gazebo

A [Gazebo](https://gazebosim.org/home) plugin to make it easy to send data to Modality from your simulator.

For more information about Modality see the [documentation.](https://docs.auxon.io/modality/)

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

## Configuration

The ModalityTracingPlugin accepts the following configuration:

- `auth_token`: Auth token hex value. For more information on Modality auth tokens see the [documentation.](https://docs.auxon.io/modality/reference/cli/user.html#modality-user-mint-auth-token)
- `timeline_name`: Name of the [timeline](https://docs.auxon.io/modality/concepts.html#events-and-timelines) for events produced by this plugin.
- `allow_insecure_tls`: Whether to allow insecure TLS connections. Equivalent to the `allow-insecure-tls` option of the [`modality-reflector` config file.](https://docs.auxon.io/modality/ingest/modality-reflector-configuration-file.html)
- `modalityd_url`: URL of the modalityd daemon to send data to. Equivalent to the `protocol-parent-url` option of the [`modality-reflector` config file.](https://docs.auxon.io/modality/ingest/modality-reflector-configuration-file.html)