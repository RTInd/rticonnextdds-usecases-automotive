# datatypes library

This folder has example automotive data types defined in OMG IDL, set up to create
libraries of type support code for most versions of RTI Connext DDS (to include Pro,
Micro 3, Micro 2, and Micro 2 Cert) and languages (C, C++, C++11).

## Building

- Point to the location of the **RTI Connext SDK** to build for by setting the environment
  variables:

  - `RTIMEHOME` to the installed location of RTI Connext Micro (if building for Micro | Cert)
     - For example: `export RTIMEHOME=~/rti_connext_dds-6.1.1/rti_connext_dds_micro-2.4.14`

  - `NDDSHOME` to the installed location of RTI Connext DDS Professional (if building for Pro)
     - For example: `export NDDSHOME=~/rti_connext_dds-6.1.1`

- Generate the script to build the library using:

      [DATABUS_PROJECT_DIR/]bin/build-gen.sh [<connext_sdk> [<target-arch> [<build_type>]]]

  where 
  - `<connext_sdk>` is the name that you want to use to refer to the build 
     for the selected SDK.  In the documentation, we use the names
      - `pro` | `micro` | `micro2` | `micro2cert`
     for Connext Professional, Connext Micro, Connext Micro 2, and Connext Micro 2 CERT respectively.

  - `<build_type>` is the type of build
    - `Debug` : to build using the debug libraries of the selected RTI Connext SDK
    - `Release` : to build using the release libraries of the selected RTI Connext SDK 

  e.g. (if building for the EVAL version of Micro CERT or Connext Drive):

      `bin/build-gen.sh micro2cert x64Linux4gcc7.3.0_certprofile Release`

  or (if building for Connext Pro):

      `bin/build-gen.sh pro x64Linux4gcc7.3.0 Debug`


- Build the repository using the generated build system script:

      build/<connext_sdk>-<target_arch>-<build_type>.sh
        
  e.g. (EVAL CERT|Drive):

      `build/micro2cert-x64Linux4gcc7.3.0_certprofile-Release.sh`

  or (pro):

      `build/pro-x64Linux4gcc7.3.0-Debug.sh`


- You can repeat the above steps for any available combinations of 
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


# `DATABUS_PROJECT_DIR`  

This project may be used as a sub-project of other projects.

In the documentation, the variable `DATABUS_PROJECT_DIR/` is used to refer
to the path of this *datatypes* project directory (i.e. the directory
containing this [README](./README.md) file).

When the working directory is not the same as the datatypes project directory,
the appropriate `DATABUS_PROJECT_DIR` path should be used. It may  be a
relative path or absolute path.

For example:

    # when the working directory is the 'datatypes' project
    DATABUS_PROJECT_DIR = .

    # when the working directory contains the 'datatypes' project as sub-directory
    DATABUS_PROJECT_DIR = datatypes

    # when the 'datatypes' project is two levels above the working directory
    DATABUS_PROJECT_DIR = ../../datatypes


