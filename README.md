# automotive-examples
Automotive examples using RTI Connext

This is a set of automotive architectural examples using RTI Connext Drive.  
The source and cmake files for the applications in the examples are created
by a generator, as guided by a system description JSON file.

The repository is in 3 parts:
 - `datatypes`: Data type definitions used in the examples, buildable as a type support library.
 - `appsrcgen`: A helper utility (Python 3) to create the example application source and build files.
 - `examples`: The buildable examples.
 
The examples can be created (using `appsrcgen`) to use RTI Connext `pro`, `micro3`, or `micro2`, in either C or C++.

Please review the individual README files in each directory for more information.

## Quick Start

After cloning this repo:

**Build the Data Types Library**

- Open a terminal in the `datatypes` directory.
- Set up the environment for your planned Connext type (pro, micro3, micro2):
    - **Pro:** set `NDDSHOME` to point to your Connext Pro installation location.
    - **Micro:** set `RTIMEHOME` to point to your Connext Micro installation location
- Run the build generator to create a build script for your Connext/Target/Build, for example:
    - `./bin/build-gen.sh micro2 x64Linux4gcc7.3.0 Release`
    - Note: change the above to fit your needs: pro/micro/micro2, different targets, Debug/Release
- Run the generated script to build the library, for example:
    - `build/micro2-x64Linux4gcc7.3.0-Release.sh`
    - Note: this will generate and build the typesupport header files and library.

**Run the Application Source Generator**

- Open a terminal in the `appsrcgen` directory.
- Run the application source generator using Python 3 and a selected JSON configuration file, for example:
    - `python3 appgen.py examples/demo_system.json`
    - **Note:** this will place the resulting source and cmake files into the `dest-path` location specified in the JSON file.  This can be changed by editing the above file, or can be overridden by specifying a path as arg[3] when running the above python application.

**Build the Example Applications**

- Open a terminal in the (newly created) 'examples' directory.
- Set up the environment for your planned Connext type (pro, micro3, micro2):
    - **Pro:** set `NDDSHOME` to point to your Connext Pro installation location.
    - **Micro:** set `RTIMEHOME` to point to your Connext Micro installation location,  
       AND set `RTIME_TARGET_NAME` to your target architecture, such as `x64Linux4gcc7.3.0`
- Create a build directory & use CMake to create the makefile:

```
    mkdir build
    cd build
    cmake ..
    make
```
**Run the Example Applications**

For convenience, a launcher script is also generated and placed in the `dest-path` location
which can be used to launch all applications in the example:
- Open a terminal in the `dest-path` location (by default this is the top-level `examples` directory).

```
    ./run-demo.sh
```

A series of terminal tabs will open (1 for each application in the example).  
Applications can be individually exited using ctrl-C.
