# automotive-examples
Automotive examples using RTI Connext Drive / Connext DDS Micro Cert.

This is a growing set of automotive architectural examples using RTI Connext DDS Micro Cert, 
which is included in [Connext Drive 2.0](https://www.rti.com/drive) or available as a
[standalone product](https://www.rti.com/products/connext-dds-cert).
Connext Drive / Micro Cert is [TUV SUD-certified to ASIL-D](https://www.rti.com/hubfs/_Products/Connext%20Drive/TUV_SUD_2022_26262_Certificate-Full.png) 
to meet the safety lifecycle requirements set forth by ISO 26262.

These examples create a working system architecture of independent applications, 
interconnected using high-performance data topics with Connext Cert.
The applications themselves are skeletal and do not contain any application logic
other than publishing topic data once per second, and printing a status message
when topic data is received.

This example is divided into 2 sections which are to be built in the following order:

 1. `datatypes`: Data type definitions used in the examples, buildable as a type support library.  This uses the RTI code generator to convert IDL data type definition files into source code to support each data type.
 2. `examples`: The buildable examples.
 
Individual README files can be found in each directory, or the examples can be built using the following Quick Start instructions:

## Quick Start

After cloning this repo:

**Build the Connext Micro Cert Library**

NOTE: if using this example with Connext Drive EVAL edition, skip ahead to the next step.
The EVAL edition has pre-built `Release` libraries for Connext Micro Cert, these are located
in the `rti_connext_dds_micro-2.4.14/lib` directories as `*target*_certprofile`, where each
target directory contains one library file ( `librti_mez.a` ).

For those with a non-EVAL installation of Connext Micro Cert, build the libraries with
`Release` build type, `RTIME_CERT=1`, and name the target using a `_certprofile` suffix,
as in: `x64Linux4gcc7.3.0_certprofile`.  
Complete build instructions are in the Connext Micro documentation.


**Build the Data Types Library**

- Open a terminal in the `datatypes` directory.
- Set up the environment for Connext Micro v2 Cert:
    - `export RTIMEHOME=<install-location-for-connext-micro2>`
- Set the target name as an environment variable, such as:
    - x64Linux:
        - `export RTIME_TARGET_NAME=x64Linux4gcc7.3.0_certprofile`
    - armv8Linux:
        - `export RTIME_TARGET_NAME=armv8Linux4gcc7.3.0_certprofile`

- If cross-compiling to a different CPU architecture (such as building ARMv8 on an x64 host PC), ensure that the correct toolchain is available.  For example:
    - armv8Linux:
        - `export CC="/usr/bin/aarch64-linux-gnu-gcc"`

- Run the build generator to create a build script for your target, such as:
    - x64Linux:
        - `./bin/build-gen.sh micro2cert x64Linux4gcc7.3.0_certprofile Release`
    - armv8Linux:
        - `./bin/build-gen.sh micro2cert armv8Linux4gcc7.3.0_certprofile Release`
    - Note: change the above to match your target platform if not listed

- Run the generated script to build the library, for example:
    - x64Linux:
        - `build/micro2cert-x64Linux4gcc7.3.0_certprofile-Release.sh`
    - armv8Linux:
        - `build/micro2cert-armv8Linux4gcc7.3.0_certprofile-Release.sh`
    - Note: this step will generate and build the typesupport header files and library.


**Build the Example Applications**

- Open a terminal in the `examples` directory.

- Set up the environment for Connext Micro v2:
    - `export RTIMEHOME=<install-location-for-connext-micro2>`

- Set the target name as an environment variable, such as:
    - x64Linux:
        - `export RTIME_TARGET_NAME=x64Linux4gcc7.3.0_certprofile`
    - armv8Linux:
        - `export RTIME_TARGET_NAME=armv8Linux4gcc7.3.0_certprofile`

- If cross-compiling to a different CPU architecture (such as building ARMv8 on an x64 host PC), ensure that the correct toolchain is available.  For example:
    - armv8Linux:
        - `export CC="/usr/bin/aarch64-linux-gnu-gcc"`

- Create a build directory:
    - `mkdir build`
    - `cd build`

- Use CMake to create the makefile for a `Release` build:
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`

- Run 'make' to build the applications:
    - `make`


**Run the Example Applications**

RUN ON LINUX BUILD PC (Native):  
If running this example in-place as native apps, a launcher script is available at `examples/run-demo.sh`,
this can be used to launch all applications in the example in separate GNOME terminal tabs:

```
    ./run-demo.sh
```
A series of terminal tabs will open (1 for each application in the example), 
and messages should appear in each terminal to indicate successful communications.  
Applications can be individually exited using ctrl-C.

RUN ON LINUX REMOTE TARGET (No desktop GUI):  
To run on a remote target such as an Automotive SoC platform running ARMv8 Linux:  

- Copy the resulting executables from the `build/<target>' directory to the remote target.
- Launch all 5 applications to run the example system


**Results**

Each application will print a message when it publishes a topic sample, and when a 
subscribed message sample is received.   At startup, status messages will also be 
printed to the terminal when connection to a publisher or subscriber has been established.  
The results should be similar to:

```
    Matched a ObjectsDetected subscriber
    Matched a Lidar publisher
    Matched a Camera publisher
    Matched a MapData publisher
    Matched a Pose3D publisher
    Matched a ObjectsDetected subscriber
    Matched a Trajectory publisher
    Matched a ObjectsDetected subscriber
    Valid Lidar_sample received
    Valid Camera_sample received
    Valid MapData_sample received
    Valid Trajectory_sample received
    Valid Pose3D_sample received
    Written ObjectsDetected sample 2
     Subscriber sleeping for 1000 msec...
    Valid Lidar_sample received
```

**Going Further**  
You now have a buildable, working, high-performance system of distributed applications
built to use the safety-certifiable Connext Cert framework.
This is an ideal starting point for building next-gen E/E vehicle architectures;
just add your own application logic to each module and take full advantage of the
communications framework that is already in place.
