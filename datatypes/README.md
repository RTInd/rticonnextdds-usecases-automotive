
# `DATABUS_PROJECT_DIR`

This databus project may be used as a sub-project of other projects.

In the documentation, the variable `DATABUS_PROJECT_DIR/` is used to refer
to the path of this *databus* project directory (i.e. the directory
containing this [README](./README.md) file).

When the working directory is not the same as the databus project directory,
the appropriate `DATABUS_PROJECT_DIR` path should be used. It may  be a
relative path or absolute path.

For example:

    # when the working directory is the 'databus' project
    DATABUS_PROJECT_DIR = .

    # when the working directory contains the 'databus' project as sub-directory
    DATABUS_PROJECT_DIR = databus

    # when the 'databus' project is two levels above the working directory
    DATABUS_PROJECT_DIR = ../../databus


## Building

- Specify the **RTI Connext SDK** to build for, by setting the environment
  variables

  - `NDDSHOME` to the location of the RTI Connext DDS Professional
  - `RTIMEHOME` to the location of the RTI Connext Micro
     - needed only if building components for Connext DDS Micro

- Generate the build system

      [DATABUS_PROJECT_DIR/]bin/build-gen.sh [<connext_sdk> [<target-arch> [<build_type>]]]

  where 
  - `<connext_sdk>` is the name that you want to use to refer to the build 
     for the selected SDK.  In the documentation, we use the names
      - `pro` | `micro` | `micro2`
     for Connext Professional, Connext Micro, and Connext Micro 2 respectively.
  - `<build_type>` is the type of build
    - `Debug` : to build using the debug libraries of the selected RTI Connext SDK
    - `Release` : to build using the release libraries of the selected RTI Connext SDK 

  e.g. (default)

      bin/build-gen.sh pro x64Linux4gcc7.3.0 Debug
  or

      databus/bin/build-gen.sh pro x64Linux4gcc7.3.0 Debug

   *NOTE:* The `build/` directory is creatd relative to the 
    *working project directory* from where the `build-gen.sh` script was 
    invoked (maybe the *top-level* project or an individual project).

- Build the repository using the generated build system

      build/<connext_sdk>-<target_arch>-<build_type>.sh
        
  e.g. (pro):

      build/pro-x64Linux4gcc7.3.0-Debug.sh

- You can repeat the above steps for any desired combinations of 
  `(connext_sdk, target_arch, build_type)`

   - An isolated build tree is generated for each combination
         
         build/<connext_sdk>/<target_arch>/<build_type>/
   
   - A standalone build script is generated for each combination
         
         build/<connext_sdk>-<target_arch>-<build_type>.sh

   - All the combinations can co-exist 

- Cleaning the generated build system

   - Delete the `build` directory to remove ALL the generated build systems
      
         rm -rf build/

   - Remove a specific combination of the build tree by removing a specific 
     `build/<connext_sdk>/<target_arch>/<build_type>` tree, e.g.:

         rm -rf build/pro/x64Linux4gcc7.3.0/Debug


## Running

Once the type support libraries are built (see [Building](#building)), they can be used
in other applications by #including the generated header files for the desired type(s),
and linking the application with the above type support libraries.

