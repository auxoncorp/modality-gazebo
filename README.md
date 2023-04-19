# modality-gazebo

A [Gazebo](https://gazebosim.org/home) plugin to make it easy to send data to Modality from your simulator.

This plugin enables tracing of entities within a gazebo simulator world. You can enable it by adding the plugin and configuration to an SDF world file, like in the [example](examples/world.sdf#L92-#L105).

The plugin provides tracing for an individual link, which will be assigned a timeline in Modality. Events are logged onto the timeline at every time step, unless the parent model is static, in which case a single event of each kind is logged.

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

The ModalityTracingPlugin accepts the following general configuration:

- `<link_name>chassis</link_name>`: Name of the link to be traced.
- `<timeline_name>robot-chassis</timeline_name>`: Name of the [timeline](https://docs.auxon.io/modality/concepts.html#events-and-timelines) for events produced by this plugin.
- `<auth_token>AUTH_TOKEN_HEX</auth_token>`: Auth token hex value. For more information on Modality auth tokens see the [documentation.](https://docs.auxon.io/modality/reference/cli/user.html#modality-user-mint-auth-token) Can alternatively be provided with the `MODALITY_AUTH_TOKEN` environment variable.
- `<allow_insecure_tls>true</allow_insecure_tls>`: Whether to allow insecure TLS connections. Equivalent to the `allow-insecure-tls` option of the [`modality-reflector` config file.](https://docs.auxon.io/modality/ingest/modality-reflector-configuration-file.html)
- `<modalityd_url>modality-ingest://localhost:14182</modality_url>`: URL of the modalityd daemon to send data to. Equivalent to the `protocol-parent-url` option of the [`modality-reflector` config file.](https://docs.auxon.io/modality/ingest/modality-reflector-configuration-file.html)

In addition, the following options configure which events the plugin should produce:

- `<pose>true</pose>`: Log pose events with x, y, z, roll, pitch, yaw attributes.
- `<linear_acceleration>true</linear_acceleration>`: Log linear acceleration events with x, y, z attributes.
- `<linear_velocity>true</linear_velocity>`: Log velocity events with x, y, z attributes.
- `<contact_collision>true</contact_collision>`: Log contact collision events with entity and name attributes.
