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

#pragma once
#include "usbdevice.h"
#include "opc.h"
#include <set>
#include <sstream>
#include <iostream>
#include <fstream>

// starts at -100 to not overlap with range of libusb_error
enum teensy4DeviceError {
    TEENSY4DEVICE_PORTNAME_NOT_READY = -100,
    TEENSY4DEVICE_PORT_WONT_OPEN = -101,
    TEENSY4DEVICE_DEVICE_WONT_OPEN = -102
};

class Teensy4Device : public USBDevice
{
public:
    Teensy4Device(libusb_device *device, bool verbose);
    virtual ~Teensy4Device();

    static bool probe(libusb_device *device);

    virtual int open();
    virtual bool probeAfterOpening();
    virtual void loadConfiguration(const Value &config);
    virtual void writeMessage(const OPC::Message &msg);
    virtual std::string getName();
    virtual void flush();

    void writeDMXPacket();

private:
    static const uint32_t START_FRAME = 0x00000000;
    static const uint32_t END_FRAME = 0xFFFFFFFF;
    static const uint32_t BRIGHTNESS_MASK = 0xE0;

    union PixelFrame
    {
        struct
        {
            uint8_t l;
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };

        uint32_t value;
    };

    std::ofstream myfile;
    char mSerialBuffer[256];
    bool mFoundEnttecStrings;
    const Value *mConfigMap;
    PixelFrame* mFrameBuffer;
    uint32_t mNumLights;

    // buffer accessor
    PixelFrame *fbPixel(unsigned num) {
        return &mFrameBuffer[num + 1];
    }

    void opcSetPixelColors(const OPC::Message &msg);
    void opcMapPixelColors(const OPC::Message &msg, const Value &inst);
    int getNumLedsFromMap();
};
