#include <devLib.h>
#include <epicsExport.h>
#include <iocsh.h>

#include "V1290N.hpp"
#include "drvCaenV1290N.hpp"

static void poll_thread_C(void* pPvt) {
    CaenV1290N* pCaenV1290N = (CaenV1290N*)pPvt;
    pCaenV1290N->poll();
}

// TODO: make configurable
const double poll_period_sec = 0.5;

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
    // writeD16(Register::SwClear, 1);

    // Read the Firmware Revision Register
    uint16_t rev;
    readD16(Register::FirmwareRev, rev);
    int major = (rev >> 4) & 0x0F;
    int minor = rev & 0x0F;
    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n\n", major, minor, rev);

    createParam(ACQUISITION_MODE_STR, asynParamInt32, &acquisitionModeId_);
    createParam(EDGE_DETECT_MODE_STR, asynParamInt32, &edgeDetectModeId_);
    createParam(ENABLE_PATTERN_STR, asynParamUInt32Digital, &enablePatternId_);
    createParam(STATUS_STR, asynParamUInt32Digital, &statusId_);
    createParam(CONTROL_STR, asynParamUInt32Digital, &controlId_);
    createParam(WINDOW_WIDTH_STR, asynParamInt32, &windowWidthId_);
    createParam(WINDOW_OFFSET_STR, asynParamInt32, &windowOffsetId_);
    createParam(SOFTWARE_CLEAR_STR, asynParamInt32, &softwareClearId_);
    createParam(TESTREG_STR, asynParamInt32, &testregId_);
    createParam(DEV_PARAM_STR, asynParamInt32, &devParamId_);

    epicsThreadCreate("CaenV1290NPoller", epicsThreadPriorityLow,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)poll_thread_C, this);
}

bool CaenV1290N::wait_micro_handshake(uint16_t mask, uint16_t timeout) {
    while (true) {
	uint16_t hs;
	if (!readD16(Register::MicroHandshake, hs)) { return false; }
	if ( (hs & mask) == 0 && (timeout > 0) ) {
	    epicsThreadSleep(0.001);
	    timeout--;
	} else {
	    break;
	}
    }
    return (timeout > 0) ? true : false;
};

bool CaenV1290N::write_micro(uint16_t opcode, uint16_t val) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
	printf("write_micro: wait_micro_handshake returned error");
	return false;
    }
    writeD16(Register::Micro, opcode);

    if (!wait_micro_handshake(Handshake::WriteOk)) {
	printf("write_micro: wait_micro_handshake returned error");
	return false;
    }
    writeD16(Register::Micro, val);
    return true;
}

bool CaenV1290N::write_micro(uint16_t opcode) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
	printf("write_micro: wait_micro_handshake returned error");
	return false;
    }
    writeD16(Register::Micro, opcode);
    return true;
}

bool CaenV1290N::read_micro(uint16_t opcode, uint16_t& value) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
	printf("read_micro: wait_micro_handshake returned error");
	return false;
    }
    writeD16(Register::Micro, opcode);

    if (!wait_micro_handshake(Handshake::ReadOk)) {
	printf("read_micro: wait_micro_handshake returned error");
	return false;
    }
    return readD16(Register::Micro, value);
}


asynStatus CaenV1290N::writeUInt32Digital(asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == enablePatternId_) {
	if (!write_micro(Opcode::WriteEnablePattern, value)) { asyn_status = asynError; }
    } else if (function == controlId_) {
	if (!writeD16(Register::Control, value)) { asyn_status = asynError; }
    }

    if (asyn_status) {
	asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::writeUInt32Digital\n");
    }

    return asyn_status;
}

asynStatus CaenV1290N::readUInt32Digital(asynUser* pasynUser, epicsUInt32* value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    uint16_t v16 = 0;
    if (function == enablePatternId_) {
	if (!read_micro(Opcode::ReadEnablePattern, v16)) { asyn_status = asynError; }
	*value = v16;
    } else if (function == controlId_) {
	if (!readD16(Register::Control, v16)) { asyn_status = asynError; }
	*value = v16;
    } else {
	asyn_status = asynPortDriver::readUInt32Digital(pasynUser, value, mask);
    }

    if (asyn_status) {
	asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readUInt32Digital\n");
    }

    return asyn_status;
}

asynStatus CaenV1290N::readInt32(asynUser* pasynUser, epicsInt32* value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    uint16_t v16 = 0;
    if (function == edgeDetectModeId_) {
	if (!read_micro(Opcode::ReadEdgeDetectionMode, v16)) { asyn_status = asynError; };
	*value = v16;
    } else if (function == acquisitionModeId_) {
	if (!read_micro(Opcode::ReadAcquisitionMode, v16)) { asyn_status = asynError; };
	*value = v16;
    } else if (function == testregId_) {
	uint32_t v32 = 0;
	if (!readD32(Register::TestReg, v32)) {
	    printf("Failed to read from test register\n");
	    asyn_status = asynError;
	}
	*value = v32;
    } else {
	asyn_status = asynPortDriver::readInt32(pasynUser, value);
    }

    if (asyn_status) {
	asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readInt32\n");
    }

    return asynSuccess;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == edgeDetectModeId_) {
	if (!write_micro(Opcode::SetEdgeDetectionMode, value)) { asyn_status = asynError; }
    } else if (function == acquisitionModeId_) {
	if (!write_micro(value == 0 ? Opcode::SetContinuous : Opcode::SetTriggerMatch)) { asyn_status = asynError; }
    } else if (function == windowWidthId_) {
	if (!write_micro(Opcode::SetWindowWidth, value)) { asyn_status = asynError; }
    } else if (function == windowOffsetId_) {
	if (!write_micro(Opcode::SetWindowOffset, value)) { asyn_status = asynError; }
    } else if (function == softwareClearId_) {
	printf("Writing %d to software clear register\n", value);
	if (!writeD16(Register::SwClear, value)) {
	    printf("Write to software clear register failed\n");
	    asyn_status = asynError;
	}
    } else if (function == testregId_) {
	if (writeD32(Register::TestReg, value)) {
	    printf("Wrote 0x%X to test register\n", value);
	} else {
	    printf("Error writing 0x%X to test register\n", value);
	    asyn_status = asynError;
	}
    }

    // TODO: remote later. this is for testing
    else if (function == devParamId_) {
	printf("writeInt32 for devParamId_ called.\n");

	uint32_t data = 0;
	if (readD32(Register::OutBuf, data)) {
	    printf("data = %X\n", data);
	} else {
	    printf("data = %X\n", data);
	    printf("Error reading output buffer register\n");
	    asyn_status = asynError;
	}

	uint32_t testval;
	if (readD32(Register::TestReg, testval)) {
	    printf("test = %X\n", testval);
	} else {
	    printf("Error reading Test register\n");
	    asyn_status = asynError;
	}
    }


    if (asyn_status) {
	asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readInt32\n");
    }

    return asynSuccess;
}

void CaenV1290N::poll() {
    while (true) {

	lock();

	uint16_t status;
	if (readD16(Register::Status, status)) {
	    setUIntDigitalParam(statusId_, status, 0xFFFF);
	}

	callParamCallbacks();
	unlock();
	epicsThreadSleep(poll_period_sec);
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
