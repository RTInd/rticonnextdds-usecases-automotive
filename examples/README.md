# Next-Gen EE Example (Connext Micro 2, C)
Automotive architectural example using RTI Connext Micro DDS, v2 (C language)  
NOTE: this example is targeting the non-CERT version of Connext Micro v2.  

**NOTE: DO NOT START HERE**  
This is the final step in building the example, it depends on first having the type support library in `../datatypes` be pre-built.   Make sure that library has been built before building this example.

**Build the Example Applications**

- Open a terminal in this 'examples' directory.
- Set up the environment for your planned Connext type (pro, micro3, micro2).
At this time only micro2/C is supported in this example, which requires setting RTIMEHOME and RTIME_TARGET_NAME, such as:
    - `export RTIMEHOME=<your-connext-micro-install-dir>`
    - `export RTIME_TARGET_NAME=x64Linux4gcc7.3.0`
- Create a build directory:
    - `mkdir build`
    - `cd build`
- Use CMake to create the makefile
    - `cmake ..`
- Build the example using 'make'
    - `make`

This will build the example applications, placing the resulting binaries in `build/x64Linux4gcc7.3.0/` (or equivalent for your target architecture).

**Run the Example Applications**

For convenience, a launcher script is available at `examples/run-demo.sh`
which can be used to launch all applications in the example:
```
    cd examples
    ./run-demo.sh
```

A series of terminal tabs will open (1 for each application in the example) and messages will be printed to each terminal window.   Note that these applications together create a next-gen EE architectural framework; they handle the inter-process communications, but otherwise have no internal logic.  
Applications can be individually exited using ctrl-C.
