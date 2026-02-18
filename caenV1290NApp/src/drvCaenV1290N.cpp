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
#define WINDOW_WIDTH_STR "WINDOW_WIDTH"
#define WINDOW_OFFSET_STR "WINDOW_OFFSET"
#define SOFTWARE_CLEAR_STR "SOFTWARE_CLEAR"
#define STATUS_STR "STATUS"
#define DEV_PARAM_STR "DEV_PARAM"

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual void poll();
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);
    virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);

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
    int windowWidthId_;
    int windowOffsetId_;
    int softwareClearId_;
    int statusId_;
    int devParamId_;
};


static void poll_thread_C(void* pPvt) {
    CaenV1290N* pCaenV1290N = (CaenV1290N*)pPvt;
    pCaenV1290N->poll();
}

CaenV1290N::CaenV1290N(const char* portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynDrvUserMask,
                     asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask, ASYN_MULTIDEVICE, ASYN_CANBLOCK, 0,
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
    createParam(ENABLE_PATTERN_STR, asynParamUInt32Digital, &enablePatternId_);
    createParam(WINDOW_WIDTH_STR, asynParamInt32, &windowWidthId_);
    createParam(WINDOW_OFFSET_STR, asynParamInt32, &windowOffsetId_);
    createParam(SOFTWARE_CLEAR_STR, asynParamInt32, &softwareClearId_);
    createParam(STATUS_STR, asynParamInt32, &statusId_);
    createParam(DEV_PARAM_STR, asynParamInt32, &devParamId_);

    // // For testing...
    // writeD32(Register::TestReg, 0xDEADBEEF);

    epicsThreadCreate("CaenV1290NPoller", epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)poll_thread_C, this);
}


asynStatus CaenV1290N::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    if (function == enablePatternId_) {
	if (!write_micro(Opcode::WriteEnablePattern, value)) return asynError;
    }
    return asynSuccess;
}

asynStatus CaenV1290N::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    if (function == enablePatternId_) {
	if (!read_micro(Opcode::ReadEnablePattern, *value)) return asynError;
    }
    return asynSuccess;
}

asynStatus CaenV1290N::readInt32(asynUser* pasynUser, epicsInt32* value) {
    const int function = pasynUser->reason;

    if (function == edgeDetectModeId_) {
	if (!read_micro(Opcode::ReadEdgeDetectionMode, *value)) return asynError;
    } else if (function == acquisitionModeId_) {
	if (!read_micro(Opcode::ReadAcquisitionMode, *value)) return asynError;
    }

    else {
	asynPortDriver::readInt32(pasynUser, value);
    }

    return asynSuccess;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;

    if (function == edgeDetectModeId_) {
	if (!write_micro(Opcode::SetEdgeDetectionMode, value)) return asynError;
    } else if (function == acquisitionModeId_) {
	if (!write_micro(value == 0 ? Opcode::SetContinuous : Opcode::SetTriggerMatch)) return asynError;
    } else if (function == windowWidthId_) {
	if (!write_micro(Opcode::SetWindowWidth, value)) return asynError;
    } else if (function == windowOffsetId_) {
	if (!write_micro(Opcode::SetWindowOffset, value)) return asynError;
    } else if (function == softwareClearId_) {
	writeD16(Register::SwClear, 1);
    }

    // TODO: remote later. this is for testing
    else if (function == devParamId_) {
	printf("writeInt32 for devParamId_ called.\n");
	printf("Reading Output buffer\n");
	uint32_t data = readD32(Register::OutBuf);
    }

    return asynSuccess;
}

void CaenV1290N::poll() {
    while (true) {

	lock();

	uint16_t status = readD16(Register::Status);
	uint32_t testval = readD32(Register::TestReg);
	printf("status = %d\n", status);
	printf("test = %X\n", testval);
	setIntegerParam(statusId_, status);
	if (status & (1<<0)) {
	    printf("Data ready!\n");
	}
	else {
	    printf("No data\n");
	}


	callParamCallbacks();
	unlock();
	epicsThreadSleep(0.5);
    }
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
