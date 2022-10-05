# automotive-examples
Automotive examples using RTI Connext

This is a growing set of automotive architectural examples using RTI Connext Drive.  These examples can be built for different Connext versions and languages:
 - Connext Pro (C, C++, C++11)
 - Connext Micro v3 (C, C++)
 - Connext Micro v2 (C-cert, C, C++)

The source and cmake files for the applications in the `examples` folder have been preconfigured for Connext Micro v2/C language; additional versions will be posted in the near future.  

These examples have further been divided into separate projects for the data type support (as a library), and the applications themselves.  These are contained in the following directories:

 - `datatypes`: Data type definitions used in the examples, buildable as a type support library.
 - `examples`: The buildable examples.
 
Individual README files can be found in each directory, or the examples can be built using the following Quick Start instructions:

## Quick Start

After cloning this repo:

**Build the Data Types Library**

- Open a terminal in the `datatypes` directory.
- Set up the environment for Connext Micro v2:
    - `export RTIMEHOME=<install-location-for-connext-micro2>`
- Run the build generator to create a build script:
    - `./bin/build-gen.sh micro2 x64Linux4gcc7.3.0 Debug`
    - Note: change the above if your target platform is different
- Run the generated script to build the library, for example:
    - `build/micro2-x64Linux4gcc7.3.0-Debug.sh`
    - Note: this will generate and build the typesupport header files and library.
- Repeat the above 2 steps, substituting `Release` for `Debug` to create a release (optimized) version of the type support library.

**Build the Example Applications**

- Open a terminal in the `examples` directory.
- Set up the environment for Connext Micro v2:
    - `export RTIMEHOME=RTIMEHOME=<install-location-for-connext-micro2>`
    - `export RTIME_TARGET_NAME=x64Linux4gcc7.3.0` 
        - Note: change the above if using a different target
- Create a build directory:
    - `mkdir build`
    - `cd build`
- Use CMake to create the makefile:
    - `cmake ..`
- Run 'make' to build the applications:
    - `make`


**Run the Example Applications**

For convenience, a launcher script is available at `examples/run-demo.sh`,
this can be used to launch all applications in the example

```
    ./run-demo.sh
```

A series of terminal tabs will open (1 for each application in the example), 
and messages should appear in each terminal to indicate successful communications.  
Applications can be individually exited using ctrl-C.
