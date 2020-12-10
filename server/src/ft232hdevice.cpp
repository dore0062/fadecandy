/*
 * Fadecandy driver for the FT232H with 8x parallel SPI driving circuit
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

#include "ft232hdevice.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "opc.h"
#include <sstream>
#include <iostream>

Ft232hDevice::Transfer::Transfer(Ft232hDevice *device, void *buffer, int length)
    : transfer(libusb_alloc_transfer(0)), finished(false)
{
    libusb_fill_bulk_transfer(transfer, device->mHandle,
        OUT_ENDPOINT, (uint8_t*) buffer, length, Ft232hDevice::completeTransfer, this, 2000);
}

Ft232hDevice::Transfer::~Transfer()
{
    libusb_free_transfer(transfer);
}

Ft232hDevice::Ft232hDevice(libusb_device *device, bool verbose)
    : USBDevice(device, "ft232h", verbose),
      mFoundUsbStrings(false),
      mConfigMap(0)
{
    mSerialBuffer[0] = '\0';
    mSerialString = mSerialBuffer;

    mConfigMapParallelMode = false;

    memset(mFrameBufferParallel, 0, sizeof(PixelFrame*)*8);
    mFrameBufferParallelCombined = NULL;
}

Ft232hDevice::~Ft232hDevice()
{
    /*
     * If we have pending transfers, cancel them.
     * The Transfer objects themselves will be freed once libusb completes them.
     */

    for (std::set<Transfer*>::iterator i = mPending.begin(), e = mPending.end(); i != e; ++i) {
        Transfer *fct = *i;
        libusb_cancel_transfer(fct->transfer);
    }

    if(mFrameBufferParallelCombined)
        free(mFrameBufferParallelCombined);

    for(unsigned i=0; i<8; i++) {
        if(mFrameBufferParallel[i])
            free(mFrameBufferParallel[i]);
    }
}

bool Ft232hDevice::probe(libusb_device *device)
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

    bool ft232hMatch = false;

    // FT232H Default
    if(dd.idVendor == 0x0403 && dd.idProduct == 0x6014)
        ft232hMatch = true;

    // TODO: add custom PID/VID?

    return ft232hMatch;
}

int Ft232hDevice::open()
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

        if (dd.iManufacturer && dd.iProduct) {
            char manufacturer[256];
            char product[256];

            r = libusb_get_string_descriptor_ascii(mHandle, dd.iManufacturer, (uint8_t*)manufacturer, sizeof manufacturer);
            if (r < 0) {
                return r;
            }

            r = libusb_get_string_descriptor_ascii(mHandle, dd.iProduct, (uint8_t*)product, sizeof product);
            if (r < 0) {
                return r;
            }

            // TODO: look for custom manufacturer/product name?
            //mFoundUsbStrings = !strcmp(manufacturer, "FTDI") && !strcmp(product, "Single RS232-HS");
            mFoundUsbStrings = !strcmp(manufacturer, "FTDI");
        }

        /*
         * Only go further if we have in fact found evidence that this is the right device.
         */

        if (mFoundUsbStrings) {

            // Only relevant on linux; try to detach the FTDI driver.
            libusb_detach_kernel_driver(mHandle, 0);

            r = libusb_claim_interface(mHandle, 0);
            if (r < 0) {
                return r;
            }

            r = libusb_get_string_descriptor_ascii(mHandle, dd.iSerialNumber,
                (uint8_t*)mSerialBuffer, sizeof mSerialBuffer);
            if (r < 0) {
                return r;
            }
        }

        return 0;
}

bool Ft232hDevice::probeAfterOpening()
{
    // By default, any device is supported by the time we get to opening it.
    return mFoundUsbStrings;
}

int Ft232hDevice::getNumLedsFromMap(const Value &inst)
{
    const Value &map = inst;
    unsigned largestIndex = 0;

    // handle a nested map differently, call this function again with the nested maps
    const Value &nestedMap = map[0u];
    if(nestedMap[0u].IsArray() && nestedMap[0u].Size() == 4) {
        std::clog << "nestedMap is array of arrays" << "\n";

        unsigned parallelLargestIndex = 0;

        for(unsigned i = 0, e = map.Size(); i != e; i++) {
            unsigned largestIndexInParallelChain = getNumLedsFromMap(map[i]);

            if(largestIndexInParallelChain > parallelLargestIndex)
                parallelLargestIndex = largestIndexInParallelChain;

            std::clog << "largestIndexInParallelChain[" << i << "]: " << largestIndexInParallelChain << "\n";
        }

        std::clog << "parallelLargestIndex: " << parallelLargestIndex << "\n";
        return parallelLargestIndex;
    }

    for (unsigned i = 0, e = map.Size(); i != e; i++) {
        const Value &inst = map[i];

        if (inst.IsArray() && inst.Size() == 4) {
            // Map a range from an OPC channel to our framebuffer

            const Value &vChannel = inst[0u];
            const Value &vFirstOPC = inst[1];
            const Value &vFirstOut = inst[2];
            const Value &vCount = inst[3];

            if (vChannel.IsUint() && vFirstOPC.IsUint() && vFirstOut.IsUint() && vCount.IsInt()) {
                unsigned firstOut = vFirstOut.GetUint();
                int count = vCount.GetInt();
                unsigned largestIndexInEntry;

                if(count > 0)
                    largestIndexInEntry = firstOut + count;
                else
                    largestIndexInEntry = firstOut;

                if(largestIndexInEntry > largestIndex)
                    largestIndex = largestIndexInEntry;

                std::clog << "largestIndexInEntry: " << largestIndexInEntry << "\n";
            }
        }
    }
    std::clog << "largestIndex: " << largestIndex << "\n";
    return largestIndex;
}

void Ft232hDevice::loadConfiguration(const Value &config)
{
    mConfigMap = findConfigMap(config);

    // at this point, we have the map and can know how many LEDs to send out
    mNumLights = 0;

    if (!mConfigMap) {
        // No mapping defined yet. This device is inactive.
        return;
    }

    const Value &map0 = *mConfigMap;
    const Value &map = map0[0u];
    if(map[0u].IsArray() && map[0u].Size() == 4) {
        std::clog << "Map is array of arrays" << "\n";
        mConfigMapParallelMode = true;
    }

    mNumLights = getNumLedsFromMap(*mConfigMap);
}

std::string Ft232hDevice::getName()
{
    std::ostringstream s;
    s << "FT232H";
    if (mSerialString[0]) {
        s << " (Serial# " << mSerialString << ")";
    }
    return s.str();
}

void Ft232hDevice::submitTransfer(Transfer *fct)
{
    /*
     * Submit a new USB transfer. The Transfer object is guaranteed to be freed eventually.
     * On error, it's freed right away.
     */

    int r = libusb_submit_transfer(fct->transfer);

    if (r < 0) {
        if (mVerbose && r != LIBUSB_ERROR_PIPE) {
            std::clog << "Error submitting USB transfer: " << libusb_strerror(libusb_error(r)) << "\n";
        }
        delete fct;
    } else {
        mPending.insert(fct);
    }
}

void Ft232hDevice::completeTransfer(struct libusb_transfer *transfer)
{
    Ft232hDevice::Transfer *fct = static_cast<Ft232hDevice::Transfer*>(transfer->user_data);
    fct->finished = true;
}

void Ft232hDevice::flush()
{
    // Erase any finished transfers

    std::set<Transfer*>::iterator current = mPending.begin();
    while (current != mPending.end()) {
        std::set<Transfer*>::iterator next = current;
        next++;

        Transfer *fct = *current;
        if (fct->finished) {
            mPending.erase(current);
            delete fct;
        }

        current = next;
    }
}

void Ft232hDevice::writeDMXPacket()
{
    /*
     * Asynchronously write the APA102 data out to serial
     *
     * TODO: We should probably throttle this so that we don't send messages
     *      faster than the device can keep up!
     */
    if(!mConfigMapParallelMode) {
        submitTransfer(new Transfer(this, (char*)&mFrameBufferParallel[0][0], sizeof(PixelFrame) * (mNumLights + 2)));
    } else {
        // assemble mFrameBufferParallelCombined
        unsigned index = 0;
        for(int i=0; i<mNumLights + 2; i++) {
            uint32_t inWord[8];
            uint8_t outByte;

            // load 8 parallel words, in the same order they'll be shifted out in parallel
            for(int j=0; j<8; j++) {
                inWord[j] = (mFrameBufferParallel[j][i].gbc << 24) |
                            (mFrameBufferParallel[j][i].r << 16) |
                            (mFrameBufferParallel[j][i].g << 8) |
                            (mFrameBufferParallel[j][i].b << 0);
            }

            // write data for 8 parallel pixels across 32 bytes
            for(int j=0; j<32; j++) {
                outByte = 0;
                for(int k=0; k<8; k++) {
                    if(inWord[k] & (1<<31))
                        outByte |= (1 << k);
                    inWord[k] <<= 1;
                }

                mFrameBufferParallelCombined[index++] = outByte;
            }
        }

        submitTransfer(new Transfer(this, (char*)&mFrameBufferParallelCombined[0], (sizeof(PixelFrame) * (mNumLights + 2))*8));
    }
}

// TODO: add custom FCDevice::writeMessage(const OPC::Message &msg) to get "device_pixels" message like fcdevice?

void Ft232hDevice::writeMessage(const OPC::Message &msg)
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

void Ft232hDevice::opcSetPixelColors(const OPC::Message &msg)
{
    /*
     * Parse through our device's mapping, and store any relevant portions of 'msg'
     * in the framebuffer.
     */

    if (!mConfigMap) {
        // No mapping defined yet. This device is inactive.
        return;
    }

    if(!mNumLights) {
        std::clog << "mNumLights == 0\n";
        return;
    }

    // the first time this runs after the map has been set, the framebuffer(s) needs to be allocated
    if(!mFrameBufferParallel[0]) {
        if(!mConfigMapParallelMode) {
            // allocate 1x size framebuffer
            uint32_t bufferSize = sizeof(PixelFrame) * (mNumLights + 2); // Number of lights plus start and end frames
            mFrameBufferParallel[0] = (PixelFrame*)malloc(bufferSize);

            std::clog << "mFrameBufferParallel[0]: " << mFrameBufferParallel[0] << "\n";

            memset(mFrameBufferParallel[0], 0, bufferSize);

            // Initialize start and end frames
            mFrameBufferParallel[0][0].littleEndianValue = START_FRAME;
            mFrameBufferParallel[0][mNumLights + 1].littleEndianValue = END_FRAME;

            // fill buffer with black
            for(int i=0; i<mNumLights; i++) {
                PixelFrame *outPtr = fbPixelParallel(0, i);
                outPtr->r = 0x00;
                outPtr->g = 0x00;
                outPtr->b = 0x00;
                outPtr->gbc = 0xFF;
            }
        } else {
            // allocate 8x size framebuffer, and 8x parallel frames
            uint32_t bufferSize = (sizeof(PixelFrame) * (mNumLights + 2))*8; // Number of lights plus start and end frames
            mFrameBufferParallelCombined = (uint8_t*)malloc(bufferSize);

            std::clog << "mFrameBufferParallelCombined: " << (void*)mFrameBufferParallelCombined << "\n";

            memset(mFrameBufferParallelCombined, 0, bufferSize);

            // Initialize start and end frames

            // set first 32 bytes to 0xFF for start frame across 8 channels
            for(unsigned i=0; i<32; i++) {
                mFrameBufferParallelCombined[i] = 0xFF;
            }

            // set last 32 bytes to 0x00 for start frame across 8 channels
            for(unsigned i=0; i<32; i++) {
                mFrameBufferParallelCombined[i + (mNumLights+1)*8] = 0x00;
            }

            // fill buffer with black
            for(int i=0; i<mNumLights; i++) {
                unsigned pixelFrameIndex = (i+1)*32;

                // set first GBC bits to 0xFF across 8 channels
                for(int j=0; j<8; j++) {
                    mFrameBufferParallelCombined[pixelFrameIndex + j] = 0xFF;
                }

                // remaining bits are already 0x00 for black, no need to set
            }

            // allocate 8x parallel frames
            bufferSize = sizeof(PixelFrame) * (mNumLights + 2);

            for(int i=0; i<8; i++) {
                mFrameBufferParallel[i] = (PixelFrame*)malloc(bufferSize);

                std::clog << "mFrameBufferParallel[" << i << "]: " << mFrameBufferParallel[i] << "\n";

                memset(mFrameBufferParallel[i], 0, bufferSize);

                // Initialize start and end frames
                mFrameBufferParallel[i][0].littleEndianValue = START_FRAME;
                mFrameBufferParallel[i][mNumLights + 1].littleEndianValue = END_FRAME;

                // fill buffer with black
                for(int j=0; j<mNumLights; j++) {
                    PixelFrame *outPtr = fbPixelParallel(i, j);
                    outPtr->r = 0x00;
                    outPtr->g = 0x00;
                    outPtr->b = 0x00;
                    outPtr->gbc = 0xFF;
                }
            }
        }
    }

    const Value &map = *mConfigMap;

    if(!mConfigMapParallelMode) {
        for (unsigned i = 0, e = map.Size(); i != e; i++) {
            opcMapPixelColors(msg, map[i], 0);
        }
    } else {
        // loop through 
        // TODO: limit to 8 channels to avoid exceeding memory bounds, elsewhere too
        for (unsigned j = 0, e1 = map.Size(); j != e1; j++) {
            const Value &nestedMap = map[j];

            for (unsigned i = 0, e = nestedMap.Size(); i != e; i++) {
                opcMapPixelColors(msg, nestedMap[i], j);
            }
        }
    }
}

void Ft232hDevice::opcMapPixelColors(const OPC::Message &msg, const Value &inst, unsigned parallelChannel)
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
                PixelFrame *outPtr = fbPixelParallel(parallelChannel, outIndex);
                outIndex += direction;
                outPtr->r = inPtr[0];
                outPtr->g = inPtr[1];
                outPtr->b = inPtr[2];
                outPtr->gbc = 0xFF; // todo: fix so we actually pass brightness
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
