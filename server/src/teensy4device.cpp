/*
 * Fadecandy driver for the Enttec DMX USB Pro.
 * 
 * Copyright (c) 2013 Micah Elizabeth Scott
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "teensy4device.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "opc.h"
#include <sstream>
#include <iostream>
#include <fstream>

// TODO: do we need flush?
// TODO: add describe() to look for numLights?

// TODO: pass in numLights to constructor to malloc correct size buffer (fcdevice and DMX don't need this as they are assumed to have the same size buffer)
//const int NUM_LIGHTS = (128*64);
const int NUM_LIGHTS = (128*64);

Teensy4Device::Teensy4Device(libusb_device *device, bool verbose)
    : USBDevice(device, "enttec", verbose),
      mFoundEnttecStrings(false),
      mConfigMap(0)
{
    mNumLights = NUM_LIGHTS;

    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;

    uint32_t bufferSize = sizeof(PixelFrame) * (mNumLights + 2); // Number of lights plus start and end frames
    mFrameBuffer = (PixelFrame*)malloc(bufferSize);

    memset(mFrameBuffer, 0, bufferSize);

    // Initialize start and end frames
    mFrameBuffer[0].value = START_FRAME;
    mFrameBuffer[mNumLights + 1].value = END_FRAME;

    // fill buffer with black
    for(int i=0; i<mNumLights; i++) {
        PixelFrame *outPtr = fbPixel(i);
        outPtr->r = 0x00;
        outPtr->g = 0x00;
        outPtr->b = 0x00;
        outPtr->l = 0xFF; // todo: fix so we actually pass brightness
    }
}

Teensy4Device::~Teensy4Device()
{
    if (myfile.is_open())
        myfile.close();

    free(mFrameBuffer);
}

bool Teensy4Device::probe(libusb_device *device)
{
    /*
     * Prior to opening the device, all we can do is look for an FT245 device.
     * We'll take a closer look in probeAfterOpening(), once we can see the
     * string descriptors.
     */

    libusb_device_descriptor dd;

    if (libusb_get_device_descriptor(device, &dd) < 0) {
        // Can't access descriptor?
        return false;
    }

    //return dd.idVendor == 0x16c0 && dd.idProduct == 0x0483; // Teensy 4.0 single sieral
    return dd.idVendor == 0x16c0 && dd.idProduct == 0x048B; // Teensy 4.0 dual sieral
}

int Teensy4Device::open()
{
    libusb_device_descriptor dd;
    int r = libusb_get_device_descriptor(mDevice, &dd);
    if (r < 0) {
        return r;
    }

    r = libusb_open(mDevice, &mHandle);
    if (r < 0) {
        return r;
    }

    /*
     * Match the manufacturer and product strings! This is the least intrusive way to
     * determine that the attached device is in fact an Enttec DMX USB Pro, since it doesn't
     * have a unique vendor/product ID.
     */

//    if (dd.iManufacturer && dd.iProduct && dd.iSerialNumber) {
    if (dd.iManufacturer && dd.iProduct) {
        char manufacturer[256];
        char product[256];

        r = libusb_get_string_descriptor_ascii(mHandle, dd.iManufacturer, (uint8_t*)manufacturer, sizeof manufacturer);
        if (r < 0) {
            return r;
        }
#if 1
        r = libusb_get_string_descriptor_ascii(mHandle, dd.iProduct, (uint8_t*)product, sizeof product);
        if (r < 0) {
            return r;
        }

        //mFoundEnttecStrings = !strcmp(manufacturer, "Teensyduino") && !strcmp(product, "USB Serial"); // Teensy 4.0 Single Serial
        mFoundEnttecStrings = !strcmp(manufacturer, "Teensyduino") && !strcmp(product, "Dual Serial");  // Teensy 4.0 Dual Serial
#else
        mFoundEnttecStrings = !strcmp(manufacturer, "Teensyduino");
#endif
    }

    /*
     * Only go further if we have in fact found evidence that this is the right device.
     */

    // TODO: add back in serialbuffer and search?
    if (mFoundEnttecStrings) {
#if 0
        r = libusb_get_string_descriptor_ascii(mHandle, dd.iSerialNumber,
            (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
        if (r < 0) {
            return r;
        }
#endif
        int port;

        // TODO: get this string using libusbp instead of setting manually
        myfile.open ("/dev/cu.usbmodem76633301");

        // TODO: appropriate return code
        if (!myfile.is_open())
            return -1;

        myfile << "test";
    }

    return 0;
}

// TODO: check for file open?
bool Teensy4Device::probeAfterOpening()
{
    // By default, any device is supported by the time we get to opening it.
    return mFoundEnttecStrings;
}

void Teensy4Device::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);
}

std::string Teensy4Device::getName()
{
    std::ostringstream s;
    //s << "Enttec DMX USB Pro";
    s << "Teensy 4";
    if (mSerialString[0]) {
        s << " (Serial# " << mSerialString << ")";
    }
    return s.str();
}

void Teensy4Device::flush()
{
    myfile << std::flush;
}

void Teensy4Device::writeDMXPacket()
{
    /*
     * Asynchronously write an FTDI packet containing an Enttec packet containing
     * our set of DMX channels.
     *
     * XXX: We should probably throttle this so that we don't send DMX messages
     *      faster than the Enttec device can keep up!
     */

    //submitTransfer(new Transfer(this, &mChannelBuffer, mChannelBuffer.length + 5));
    //myfile << mFrameBuffer;
    myfile.write((char*)&mFrameBuffer[0], sizeof(PixelFrame) * (mNumLights + 2));
    myfile << std::flush;
}

// TODO add custom FCDevice::writeMessage(const OPC::Message &msg) to get "device_pixels" message like fcdevice?

void Teensy4Device::writeMessage(const OPC::Message &msg)
{
    /*
     * Dispatch an incoming OPC command
     */

    switch (msg.command) {

        case OPC::SetPixelColors:
            opcSetPixelColors(msg);
            writeDMXPacket();
            return;

        case OPC::SystemExclusive:
            // No relevant SysEx for this device
            return;
    }

    if (mVerbose) {
        std::clog << "Unsupported OPC command: " << unsigned(msg.command) << "\n";
    }
}

void Teensy4Device::opcSetPixelColors(const OPC::Message &msg)
{
    /*
     * Parse through our device's mapping, and store any relevant portions of 'msg'
     * in the framebuffer.
     */

    if (!mConfigMap) {
        // No mapping defined yet. This device is inactive.
        return;
    }

    const Value &map = *mConfigMap;
    for (unsigned i = 0, e = map.Size(); i != e; i++) {
        opcMapPixelColors(msg, map[i]);
    }
}

void Teensy4Device::opcMapPixelColors(const OPC::Message &msg, const Value &inst)
{
    /*
     * Parse one JSON mapping instruction, and copy any relevant parts of 'msg'
     * into our framebuffer. This looks for any mapping instructions that we
     * recognize:
     *
     *   [ OPC Channel, OPC Pixel, Pixel Color, DMX Channel ]
     */

    unsigned msgPixelCount = msg.length() / 3;

    if (inst.IsArray() && inst.Size() == 4) {
        // Map a range from an OPC channel to our framebuffer

        const Value &vChannel = inst[0u];
        const Value &vFirstOPC = inst[1];
        const Value &vFirstOut = inst[2];
        const Value &vCount = inst[3];

        if (vChannel.IsUint() && vFirstOPC.IsUint() && vFirstOut.IsUint() && vCount.IsInt()) {
            unsigned channel = vChannel.GetUint();
            unsigned firstOPC = vFirstOPC.GetUint();
            unsigned firstOut = vFirstOut.GetUint();
            unsigned count;
            int direction;
            if (vCount.GetInt() >= 0) {
                count = vCount.GetInt();
                direction = 1;
            }
            else {
                count = -vCount.GetInt();
                direction = -1;
            }

            if (channel != msg.channel) {
                return;
            }

            // Clamping, overflow-safe
            firstOPC = std::min<unsigned>(firstOPC, msgPixelCount);
            firstOut = std::min<unsigned>(firstOut, mNumLights);
            count = std::min<unsigned>(count, msgPixelCount - firstOPC);
            count = std::min<unsigned>(count,
                direction > 0 ? mNumLights - firstOut : firstOut + 1);

            // Copy pixels
            const uint8_t *inPtr = msg.data + (firstOPC * 3);
            unsigned outIndex = firstOut;
            while (count--) {
                PixelFrame *outPtr = fbPixel(outIndex);
                outIndex += direction;
#if 1
                outPtr->r = inPtr[0];
                outPtr->g = inPtr[1];
                outPtr->b = inPtr[2];
#else
                outPtr->r = 0xFF;
                outPtr->g = inPtr[1];
                outPtr->b = inPtr[2];
#endif
                outPtr->l = 0xFF; // todo: fix so we actually pass brightness
                inPtr += 3;
            }

            return;
        }
    }

    // Still haven't found a match?
    if (mVerbose) {
        rapidjson::GenericStringBuffer<rapidjson::UTF8<> > buffer;
        rapidjson::Writer<rapidjson::GenericStringBuffer<rapidjson::UTF8<> > > writer(buffer);
        inst.Accept(writer);
        std::clog << "Unsupported JSON mapping instruction: " << buffer.GetString() << "\n";
    }
}



