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
#define ENABLE_PATTERN_STR "ENABLE_PATTERN"

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);

  private:
    volatile uint8_t* base;

    bool wait_micro_handshake(uint16_t mask) {
	uint16_t timeout = 1000;
	while ((readD16(Register::MicroHandshake) & mask) == 0 && timeout > 0) {
	    epicsThreadSleep(0.0001);
	    timeout--;
	}
	return (timeout > 0) ? true : false;
    };

    bool write_micro(uint16_t opcode, uint16_t val) {
	if(!wait_micro_handshake(Handshake::WriteOk)) {
	    printf("Timeout waiting for micro handshake write\n");
	    return false;
	}
	writeD16(Register::Micro, opcode);

	if(!wait_micro_handshake(Handshake::WriteOk)) {
	    printf("Timeout waiting for micro handshake write\n");
	    return false;
	}
	writeD16(Register::Micro, val);
	return true;
    }

    bool write_micro(uint16_t opcode) {
	if(!wait_micro_handshake(Handshake::WriteOk)) {
	    printf("Timeout waiting for micro handshake write\n");
	    return false;
	}
	writeD16(Register::Micro, opcode);
	return true;
    }

    template<typename T>
    bool read_micro(uint16_t opcode, T& retval) {
	if(!wait_micro_handshake(Handshake::WriteOk)) {
	    printf("Timeout waiting for micro handshake write\n");
	    return false;
	}
	writeD16(Register::Micro, opcode);

	if (!wait_micro_handshake(Handshake::ReadOk)) {
	    printf("Timeout waiting for micro handshake read\n");
	    return false;
	}
	retval = readD16(Register::Micro);
	return true;
    }

    void writeD16(uint16_t offset, uint16_t value) { nat_iowrite16(base + offset, value); }
    uint16_t readD16(uint16_t offset) { return nat_ioread16(base + offset); }
    void writeD32(uint32_t offset, uint32_t value) { nat_iowrite32(base + offset, value); }
    uint32_t readD32(uint32_t offset) { return nat_ioread32(base + offset); }

  protected:
    int acquisitionModeId_;
    int edgeDetectModeId_;
    int enablePatternId_;
};


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

    // Read the Firmware Revision Register
    uint16_t rev = readD16(Register::FirmwareRev);
    int major = (rev >> 4) & 0x0F;
    int minor = rev & 0x0F;
    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n\n", major, minor, rev);

    createParam(ACQUISITION_MODE_STR, asynParamInt32, &acquisitionModeId_);
    createParam(EDGE_DETECT_MODE_STR, asynParamInt32, &edgeDetectModeId_);
    createParam(ENABLE_PATTERN_STR, asynParamInt32, &enablePatternId_);
}

asynStatus CaenV1290N::readInt32(asynUser* pasynUser, epicsInt32* value) {
    const int function = pasynUser->reason;

    if (function == edgeDetectModeId_) {
	if (!read_micro(Opcode::ReadEdgeDetectionMode, *value)) return asynError;
    } else if (function == acquisitionModeId_) {
	if (!read_micro(Opcode::ReadAcquisitionMode, *value)) return asynError;
    } else if (function == enablePatternId_) {
	if (!read_micro(Opcode::ReadEnablePattern, *value)) return asynError;
    }

    return asynSuccess;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;

    if (function == edgeDetectModeId_) {
	if (!write_micro(Opcode::SetEdgeDetectionMode, value)) return asynError;
    } else if (function == acquisitionModeId_) {
	if (!write_micro(value == 0 ? Opcode::SetContinuous : Opcode::SetTriggerMatch)) return asynError;
    } else if (function == enablePatternId_) {
	if (!write_micro(Opcode::WriteEnablePattern, value)) return asynError;
    }

    return asynSuccess;
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
