# Next-Gen EE Example (Connext Micro 2, C)
Automotive architectural example using RTI Connext Micro DDS, v2 (C language)  
NOTE: this example is targeting the non-CERT version of Connext Micro v2.  


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
