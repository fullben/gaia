# Gaia

A 3D physics engine for [Processing](https://processing.org/), based on the [Cyclone physics engine](https://github.com/idmillington/cyclone-physics) by Ian Millington. It was created in order to provide users of Processing with an engine capable of supporting basic collision dynamics for simple 3D bodies. Development began as part of my bachelor's thesis in early 2018. Initially, the project went by the name of *eos*, which was changed to *Gaia* (greek primordial deity representing Earth) in early 2021.

The physics engine is a real-time engine, supporting both particles and rigid bodies. Similar to [toxiclibs](http://toxiclibs.org/about/), the engine attempts to work with a coordinate system roughly equivalent to the one used by Processing in order to minimize value conversion when providing the engine with data or retrieving it from Gaia.

In order to support both particles and rigid bodies, Gaia is made up of two smaller engines. While these two engines have very different capabilities, their structural makeup is very similar for the most part. Refer to the separate [guide document](GUIDE.md) for further details regarding structure, features and limitations of the engine.

The main focus of the engine is collision detection and resolution between simple geometric shapes - planes, spheres and boxes. The resolution follows a sequential pattern, which works well for simulations inhabited by only a few bodies. The strategy of the resolution is impulse-based, meaning bodies in resting contacts are kept in position by tiny collisions which occur during each integration step.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine. You can use this project either for building a library that can be used as a [Processing library](https://processing.org/reference/libraries/) or for development work.

### Prerequisites

Before downloading the project, you need to make sure that the following software is installed on your machine:

* Java Development Kit 11 (e.g. [AdoptOpenJDK](https://adoptopenjdk.net/))
* [Gradle 6.X](https://gradle.org/releases/)
* [Processing](https://processing.org/download/) (tested with 4.0 alpha 2)

Furthermore, if you want to actively work with the code, consider installing the following software:

* [IntelliJ IDEA](https://www.jetbrains.com/idea/download/) (or an IDE of your choice)
* [Git](https://git-scm.com/downloads)

Once all required software has been installed, it is necessary to manually copy some Processing core JARs to the project, as these are required for successful execution. This is due to the fact that there is no way to automatically pull the Processing JARs as a dependency from a repository.

Navigate to the directory Processing is installed in. In this directory, go to the `core` directory. Copy the following JARs:

* `core.jar`
* `gluegen-rt.jar`
* `gluegen-rt-natives-windows-amd64.jar` (this is the appropriate JAR for systems running a 64-bit Windows. If you are using a different system, you may need to use another JAR, e.g. `gluegen-rt-natives-macosx-universal.jar` for macOS)
* `jogl-all.jar`
* `jogl-all-natives-windows-amd64.jar` (this is the appropriate JAR for systems running a 64-bit Windows. If you are using a different system, you may need to use another JAR, e.g. `jogl-all-natives-macosx-universal.jar` for macOS)

Place them in `PROJECT_ROOT/lib/dependencies/processing-core`.

### Run the Examples

The project contains a number of Gaia usage examples implemented in Java, located in `PROJECT_ROOT/lib/src/examples`. For running any of the examples, simply execute the `main(String[] args)` method found in the implementation.

### Build Gaia as Processing Library

Use the Gradle task `processingLib` to build the project. This will generate a library build of Gaia adhering to the [Processing library guidelines](https://github.com/processing/processing/wiki/Library-Guidelines). The build result can be found in `PROJECT_ROOT/lib/build/gaia`.

Note that even though the project does feature examples, the `examples` directory of the built library will always be empty. This is due to the fact that the examples provided with the library are implemented in Java, featuring inheritance, which is difficult to map to Processing sketches.

For using the Gaia physics engine in Processing, copy the `gaia` folder to the `libraries` folder found in your [Sketchbook](https://processing.org/reference/environment/#Sketchbook).

## Usage

Check out the examples found in `PROJECT_ROOT/lib/src/examples` or refer to the [guide](GUIDE.md) for information on how to utilize the Gaia physics engine in your own projects.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

All changes should honor the [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html). Please make sure to update tests and documentation as appropriate.

## License

[MIT](LICENSE.txt)
