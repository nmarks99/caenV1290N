#include <stdlib.h>

#include <asynPortDriver.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsExport.h>
#include <epicsMMIO.h>
#include <epicsThread.h>
#include <epicsTypes.h>
#include <errMdef.h>
#include <stdint.h>
#include <iocsh.h>

#include "V1290N.hpp"

// String names for asyn parameters
#define ACQUISITION_MODE_STR "ACQUISITION_MODE"
#define EDGE_DETECT_MODE_STR "EDGE_DETECT_MODE"

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);

  private:
    volatile uint8_t* base;

    bool wait_micro_handshake(uint16_t mask);
    bool write_opcode(uint16_t mask);
    bool read_opcode(uint16_t mask);

    inline void writeD16(uint16_t offset, uint16_t value) { nat_iowrite16(base + offset, value); }
    inline uint16_t readD16(uint16_t offset) { return nat_ioread16(base + offset); }
    inline void writeD32(uint32_t offset, uint32_t value) { nat_iowrite32(base + offset, value); }
    inline uint32_t readD32(uint32_t offset) { return nat_ioread32(base + offset); }

  protected:
    int acquisitionModeId_;
    int edgeDetectModeId_;
};

// #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)
#define NUM_PARAMS 0

CaenV1290N::CaenV1290N(const char* portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, asynInt32Mask | asynFloat64Mask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask, ASYN_MULTIDEVICE, ASYN_CANBLOCK, 0,
                     0) // Default priority and stack size
{

    // initialize
    volatile void* ptr;
    const size_t EXTENT = 0x10000; // ???
    if (devRegisterAddress("CAEN_V1290N", atVMEA32, baseAddress, EXTENT, &ptr)) {
        printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
        return;
    }

    base = (volatile uint8_t*)ptr;

    // Writing any value to the Software Clear Register resets logic and clears buffers.
    writeD16(Register::SwClear, 1);
    printf("Board logic cleared.\n\n");

    // Read the Firmware Revision Register
    uint16_t rev = readD16(Register::FirmwareRev);
    int major = (rev >> 4) & 0x0F;
    int minor = rev & 0x0F;
    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n\n", major, minor, rev);

    createParam(ACQUISITION_MODE_STR, asynParamInt32, &acquisitionModeId_);
    createParam(EDGE_DETECT_MODE_STR, asynParamInt32, &edgeDetectModeId_);

    // // =====================================================================
    // // Enable test mode
    // printf("Reading control register...\n");
    // uint16_t ctrl_reg = readD16(V1290_CONTROL);
    // printf("Control register = 0x%X\n", ctrl_reg);
    // printf("Setting TEST_FIFO_ENABLE bit in control register\n");
    // ctrl_reg |= TEST_FIFO_ENABLE;
    // writeD16(V1290_CONTROL, ctrl_reg);
    // printf("Read control register = 0x%X\n\n", readD16(V1290_CONTROL));
    // // write test data into test registers
    // printf("Writing 0x%X into test register...\n", 0xDEADBEEF);
    // writeD32(V1290_TESTREG, 0xDEADBEEF);
    // printf("Read test register = 0x%X\n\n", readD32(V1290_TESTREG));
    // printf("Writing 0x%X into Dummy32 register...\n", 0xCAFEBABE);
    // writeD32(V1290_DUMMY32, 0xCAFEBABE);
    // printf("Read Dummy32 register = 0x%X\n\n", readD32(V1290_DUMMY32));
    // printf("Writing 0x%X into Dummy16 register...\n", 0xCAFE);
    // writeD16(V1290_DUMMY16, 0xCAFE);
    // printf("Read Dummy16 register = 0x%X\n\n", readD16(V1290_DUMMY16));
    // // Generate a software trigger
    // printf("Generating software trigger...\n");
    // writeD16(V1290_SW_TRIGGER, 1);
}

bool CaenV1290N::wait_micro_handshake(uint16_t mask) {
    uint16_t timeout = 1000;
    while ((readD16(Register::MicroHandshake) & mask) == 0 && timeout > 0) {
	epicsThreadSleep(0.001);
	timeout--;
    }
    printf("timeout = %d\n", timeout);
    return (timeout > 0) ? true : false;
};

asynStatus CaenV1290N::readInt32(asynUser *pasynUser, epicsInt32 *value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;


    if (function == edgeDetectModeId_) {
	if(!wait_micro_handshake(Handshake::WriteOk)) {
	    printf("Timeout waiting for micro handshake write\n");
	    return asynError;
	}
	printf("Write okay, continuing\n");
	writeD16(Register::Micro, Opcode::ReadEdgeDetectionConfig);
	printf("Write done\n");

	if (!wait_micro_handshake(Handshake::ReadOk)) {
	    printf("Timeout waiting for micro handshake read\n");
	    return asynError;
	}
	printf("Read okay, continuing\n");
	uint16_t edge_detect_mode = readD16(Register::Micro);
	printf("Micro register = 0x%X\n", edge_detect_mode);
	*value = edge_detect_mode;
    }

    return asyn_status;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == edgeDetectModeId_) {
	printf("writeInt32 called for edgeDetectModeId_\n");
    }

    return asyn_status;
}

extern "C" int initCaenV1290N(const char* portName, int baseAddress) {
    new CaenV1290N(portName, baseAddress);
    return (asynSuccess);
}

static const iocshArg initArg0 = {"Port name", iocshArgString};
static const iocshArg initArg1 = {"Base Address", iocshArgInt};
static const iocshArg* const initArgs[2] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"initCAEN_V1290N", 2, initArgs};
static void initCallFunc(const iocshArgBuf* args) { initCaenV1290N(args[0].sval, args[1].ival); }

void drvCaenV1290NRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

extern "C" {
epicsExportRegistrar(drvCaenV1290NRegister);
}
