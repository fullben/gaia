# Release Notes

All notable changes to this project will be documented in this file.

## Development

- Rename the accessor for the global library configuration object from `Configuration.getConfig()` to `Configuration.current()` and the method for updating the configuration from `Configuration.setConfig(Configuration)` to `Configuration.update(Configuration)`
- Add a step builder for a more comprehensible way of configuring and creating `RigidBody` instances representing simple objects (such as cubes or spheres)
- Move guide documents to a dedicated directory

## 0.2.0 (11.04.2021)
- Initial public release of the engine
