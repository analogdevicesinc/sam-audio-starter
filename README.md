# SAM-Audio-Starter

# Overview
This project is tailored for quick audio startup with the SAM board (ADZS-SC589-MINI). Some components in this audio starter project are derived from Open Source (OSS) projects and have BSD, MIT, or similar commercial friendly licenses.

For full details about this project, 
features and set up instructions, please refer to: https://wiki.analog.com/resources/tools-software/sharc-audio-module/advanced-audio-projects

# To build

### Install the tools
- Install CCES as-per Analog Devices instructions
- Install Git bash shell from here: https://git-scm.com/downloads
- Instead of Git bash, one can also install MinGW/MSYS2 from
  here: http://www.msys2.org/

## Standalone Compilation (By Command Line)

### Source the environment
- Check your path first to see if it already includes the desired version of
  CCES.  In general Git bash inherits the path and MinGW/MSYS2 does not
  unless it's launched with '-use-full-path' option.

```
echo $PATH  # Check inherited path
```

- If necessary, modify the 'env.sh' shell to match your CCES installation
  directory and include the environment in your shell.

```
. ./env.sh # Note the space between the first two periods!
```

### Build the code
- Go into the build directory and type `make`

```
cd build
make -j4
```
### Binaries
- Binaries are located in the root of the _build_ folder.

## Compilation through CCES (Windows only!)

- Open CCES and set preferred workspace
- In the project explorer, right-click and select _Import_ -> _General_ -> _Existing Projects into Workspace_ and select _Next_
- Change the root directory to _<project_root>_ and select _Finish_ . Note that three projects will be imported, one for each core (ARM, SHARC0 and SHARC1))
- To build in _Debug_ mode, set the active build configuration for each core to _Debug_. To build in _Release_ mode, set the active build configuraiton for each core to _Release_. 
- Binaries are located in _build/CCES/Binaries/<Debug|Release>_

## Debugging the code
- Open CCES, create a new debug configuration
- Load `<CCES_INSTALL_DIR>\SHARC\ldr\ezkitSC589_preload_core0_v01` into core0
- Additionally, load the `SAM-Audio-Starter-ARM.exe` executable into core0
- Load the `SAM-Audio-Starter-SHARC0.dxe` executable into core1
- Load the `SAM-Audio-Starter-SHARC1.dxe` executable into core2
- Under the "Automatic Breakpoint" tab, be sure to uncheck the "Enable
  semihosting" checkbox located at the bottom of the tab window.
- Save and start debugging.

## Additional Notes

- The makefiles for standalone vs. CCES are NOT the same. CCES automatically generates the makefile based on parameter settings in the tool. Updates to settings for one will not reflect in the other. 
- While the CCES project should work for any version, it has only been verified using versions 2.9.3, 2.11.0 and 2.11.1. 
- Relese notes can be found in the root of this project. 

## LICENSE and NOTICE files

For detailed component license information, please refer to the LICENSE and NOTICE files located throughout this repository.
