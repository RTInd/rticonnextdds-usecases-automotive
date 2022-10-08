# Next-Gen EE Example (Connext Micro Cert, C)

Automotive architectural example using RTI Connext Micro Cert, v2 (C language)  

**NOTE: DO NOT START HERE**

This is the final step in building the example, it depends on first having the type support
library in `../datatypes` be pre-built.   Make sure that library has been built before building this example.

**Build the Example Applications**

- Open a terminal in this 'examples' directory.
- Set up the environment for your planned Connext type (pro, micro3, micro2).
At this time only Connect Micro Cert is supported in this example, which requires setting RTIMEHOME and RTIME_TARGET_NAME, for example:
    - `export RTIMEHOME=~/rti_connext_dds-6.1.1/rti_connext_dds_micro-2.4.14`
    - `export RTIME_TARGET_NAME=x64Linux4gcc7.3.0_certprofile`

- Create a build directory:
    - `mkdir build`
    - `cd build`

- Use CMake to create the makefile, using a `Release` build type
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`

- Build the example using 'make'
    - `make`

This will build the example applications, placing the resulting binaries in `build/x64Linux4gcc7.3.0_certprofile/`
(or equivalent for your target architecture).

**Run the Example Applications**

For convenience, a launcher script is available at `examples/run-demo.sh`
which can be used to launch all applications in the example:
```
    cd examples
    ./run-demo.sh
```

A series of terminal tabs will open (1 for each application in the example) and messages will be printed to each terminal window.   Note that these applications together create a next-gen EE architectural framework; they handle the inter-process communications, but otherwise have no internal logic.  
Applications can be individually exited using ctrl-C.
