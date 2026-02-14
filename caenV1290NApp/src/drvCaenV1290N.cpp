#include <stdlib.h>

#include <asynPortDriver.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsExport.h>
#include <epicsMMIO.h>
#include <epicsThread.h>
#include <epicsTypes.h>
#include <errMdef.h>
#include <iocsh.h>

#include "V1290N.hpp"

// String names for asyn parameters
#define ACQ_MODE_STR "ACQ_MODE"

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);

  private:
    volatile uint8_t* base;

    asynStatus wait_handshake(uint16_t mask);
    asynStatus write_opcode(uint16_t code);

    inline void writeD16(uint16_t offset, uint16_t value) { nat_iowrite16(base + offset, value); }
    inline uint16_t readD16(uint16_t offset) { return nat_ioread16(base + offset); }
    inline void writeD32(uint32_t offset, uint32_t value) { nat_iowrite32(base + offset, value); }
    inline uint32_t readD32(uint32_t offset) { return nat_ioread32(base + offset); }

  protected:
    int acqModeId_;
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

    createParam(ACQ_MODE_STR, asynParamInt32, &acqModeId_);

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


asynStatus CaenV1290N::wait_handshake(uint16_t mask) {
    uint16_t timeout = 1000;
    while ((readD16(Register::MicroHandshake) & mask) == 0 && timeout > 0) {
	epicsThreadSleep(0.001);
	timeout--;
    }
    return (timeout > 0) ? asynSuccess : asynError;
}

asynStatus CaenV1290N::write_opcode(uint16_t code) {
    if (wait_handshake(Handshake::WriteOk) != asynSuccess) return asynError;
    writeD16(Register::Micro, code);
    return asynSuccess;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == acqModeId_) {
        printf("Set aquisition mode %d called\n", value);
	if (value == 0) {
	    write_opcode(Opcode::SetTriggerMatch);
	} else if (value == 1) {
	    write_opcode(Opcode::SetContinuous);
	}
    }

    if (write_opcode(Opcode::ReadAcqMode) != asynSuccess) return asynError;
    if (wait_handshake(Handshake::ReadOk) != asynSuccess) return asynError;
    uint16_t val = readD16(Register::Micro);
    printf("acqusition mode = %d\n", val & 0x1);

    return asyn_status;
}

extern "C" int initCaenV1290N(const char* portName, int baseAddress) {
    new CaenV1290N(portName, 0xB2000000); // hack
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
