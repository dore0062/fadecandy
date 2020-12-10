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

class Ft232hDevice : public USBDevice
{
public:
    Ft232hDevice(libusb_device *device, bool verbose);
    virtual ~Ft232hDevice();

    static bool probe(libusb_device *device);

    virtual int open();
    virtual bool probeAfterOpening();
    virtual void loadConfiguration(const Value &config);
    virtual void writeMessage(const OPC::Message &msg);
    virtual std::string getName();
    virtual void flush();

    void writeDMXPacket();

private:
    static const unsigned OUT_ENDPOINT = 2;
    static const uint32_t BRIGHTNESS_MASK = 0xE0;

    static const uint32_t START_FRAME = 0x00000000;
    static const uint32_t END_FRAME = 0xFFFFFFFF;

    union PixelFrame
    {
        struct
        {
            uint8_t gbc;
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };

        // careful when trying to use this, "gbc" is in the lowest bits, "b" in the highest bits
        uint32_t littleEndianValue;
    };

    struct Transfer {
        Transfer(Ft232hDevice *device, void *buffer, int length);
        ~Transfer();
        libusb_transfer *transfer;
        bool finished;
    };

    char mSerialBuffer[256];
    bool mFoundUsbStrings;
    const Value *mConfigMap;
    bool mConfigMapParallelMode;
    PixelFrame* mFrameBufferParallel[8];
    uint8_t* mFrameBufferParallelCombined;
    uint32_t mNumLights;
    std::set<Transfer*> mPending;

    PixelFrame *fbPixelParallel(unsigned channel, unsigned num) {
        return &mFrameBufferParallel[channel][num + 1];
    }

    void submitTransfer(Transfer *fct);
    static LIBUSB_CALL void completeTransfer(struct libusb_transfer *transfer);

    void opcSetPixelColors(const OPC::Message &msg);
    void opcMapPixelColors(const OPC::Message &msg, const Value &inst, unsigned parallelChannel);
    int getNumLedsFromMap(const Value &inst);
};
