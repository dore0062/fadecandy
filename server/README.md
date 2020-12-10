Fadecandy Server
================

## SmartMatrix/Teensy Fork

This fork of Fadecandy Server adds support for driving the Teensy 3.1/3.2 and Teensy 4.0/4.1 using [SmartMatrix Library](https://github.com/pixelmatix/SmartMatrix).

### SmartMatrix with Teensy 3.1/3.2 

This repo has a new device class added to the server to add support for driving a Teensy 3.1/3.2 driving a SmartMatrix display.  A new "smartmatrix" device is created, see `/examples/config/smartmatrix*.json` for examples of using the server.  The "smartmatrix" device is nearly identical to the "fadecandy" device, with unique USB IDs, and an additional USB message "TYPE_EXPAND" that sends the high bits of packet index to allow expanding beyond 32 packets per frame (32x21 pixels gives a limit of 672 pixels).  The server modifications don't affect the "fadecandy" device, so this server should be able to drive "fadecandy" devices as before, and you should be able to combine "smartmatrix" and "fadecandy" devices (untested as of yet).

SmartMatrix running on a Teensy 3.1/3.2 can support driving more pixels per device than Fadecandy (the Teensy 3.1/3.2 has more memory and CPU speed than the Teensy 3.0-based Fadecandy, and uses a different type of LED panel).  The server is configured to send 2048 pixels - enough for a 32x64 pixel panel - per frame.  You may get better performance by changing NUM_PIXELS in `/server/src/smartmatrixdevice.h` to the number of pixels in your panel.

The SmartMatrix_Fadecandy firmware for Teensy 3.1/3.2 is stored in a separate repo: https://github.com/pixelmatix/SmartMatrix_fadecandy

Note: the "smartmatrix" name was chosen when SmartMatrix Library only ran on the Teensy 3.1, and a more specific name is probably needed now that there's a Teensy 4 SmartMatrix device now.  The device may get renamed to "teensy3" something similar soon, but I'll add in backwards compatibility if possible.

With the new Teensy 4.0 being the same price as a Teensy 3.2, and being much more powerful, there's not much reason to use the Teensy 3.2 anymore for Fadecandy with SmartMatrix, so this device is unlikely to be improved in the future.

There are several Processing examples updated for running on SmartMatrix: /examples/processing/smartmatrix*
You should use ledGrid() with these parameters for driving a SmartMatrix device:  
stripLength = SmartMatrix panel width  
numStrips = SmartMatrix panel height
zigzag = false

### SmartMatrix with Teensy 4.0/4.1

This repo also has a new device class added to the server to add support for driving a Teensy 4.0/4.1 driving a SmartMatrix display.  This "teensy4" device uses a different prototcol than the Teensy 3.1/3.2 SmartMatrix device.  See `/examples/config/teensy4*.json` for examples of using the server.  The "teensy4" device is significantly different from the "fadecandy" device as it uses simple serial communication over USB CDC.  The Teensy 4 uses USB High Speed with a lot more bandwidth than the Teensy 3's USB Full Speed, so having an optimized protocol isn't as important.

The Teensy 4 has enough CPU, RAM, and USB bandwith to transfer over 10,000 pixels at 60FPS, and refresh a HUB75 panel with 36-bit color, at 240Hz refresh rate.

While this device is fairly functinoal, there are still some unfinished features:

- The "color" settings are ignored, the device just does simple color (gamma) correction using on on-device lookup table
- You can't reorder the color channels, it's fixed as RGB order (alternate RGB order is probably of limited use for this device)
- CMAKE hasn't been fully updated to compile all the latest changes (please help with this if you need CMAKE)
- There's some other minor "TODO"s listed in teensy4device.cpp

There are several Processing examples updated for running on SmartMatrix: /examples/processing/smartmatrix*
You should use ledGrid() with these parameters for driving a SmartMatrix device:  
stripLength = SmartMatrix panel width  
numStrips = SmartMatrix panel height
zigzag = false

This has been tested on one MacOS 10.15.7 machine, and likely has issues compiling on other machines and operating systems.  Please get in contact through GitHub Issues or at [SmartMatrix Community](community.pixelmatix.com) if you have trouble or can help.

### Fadecandy Server Overview

The Fadecandy Server is a background process that handles the USB communications with one or more Fadecandy Controller boards.

You can send pixel data to a Fadecandy Server over the Open Pixel Control protocol, or from a web app via WebSockets. See the 'doc' directory for details on all protocols supported.

The Fadecandy Server optionally takes configuration options in the form of a JSON config file. Configuration files allow you to do things like:

* Support multiple Fadecandy boards
* Mix Fadecandy and DMX lighting devices
* Listen on an alternate TCP port
* Listen for connections from the network, not just from local programs

The configuration file format is documented in the **doc** directory.

When you run the Fadecandy Server, it will provide a simple web interface. By default, the Fadecandy server runs at [http://localhost:7890](http://localhost:7890).

Build
-----

Pre-built binaries are included in the **bin** directory, but you can also build it yourself. All required libraries are included as git submodules.

It can build on Windows, Mac OS, or Linux using Make and other command line tools. On Windows, the build uses MinGW and gcc.


Getting Started
---------------

In order to build the binary from source you need to run the following commands inside of the **server** directory:

```bash
$ make submodules
$ make
```

The compiled binary will be created in the same **server** directory

If you want to remove the compiled binary and source files run:

```bash
$ make clean
```


Build using CMake
-----------------

The CMake project supports building a Debian package including a SystemD service. Run the following commands in the **server** directory to build using it.

To build the binaries and get a Debian package:


```bash
$ make submodules # Fetches submodule git repositories
$ mkdir build
$ cd build
$ cmake ..
$ make          # Builds the binaries.

$ cpack -G DEB  # Generates the debian package.
```

To list CMake options:

```bash
$ cmake -LH ..
```

You can also install on the system without doing it via a Debian package:
```bash
$ make install
```
