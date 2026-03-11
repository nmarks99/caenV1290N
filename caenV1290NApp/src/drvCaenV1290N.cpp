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

const int ASYN_INTERFACE_MASK = asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask;
const int ASYN_INTERRUPT_MASK = asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask;

CaenV1290N::CaenV1290N(const char* portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS,
            ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
            ASYN_MULTIDEVICE, 1, 0, 0) {

    // // initialize
    volatile void* ptr;
    const size_t EXTENT = 0x10000;
    // const size_t EXTENT = 0x1204*4; // what should this be?
    if (devRegisterAddress("CAEN_V1290N", atVMEA32, baseAddress, EXTENT, &ptr)) {
        printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
        return;
    }

    // ????
    // if (sysBusToLocalAdrs(0x09, (char*)baseAddress, (char**)ptr)) {
    // printf("ERROR: sysBusToLocalAdrs failed. Cannot initialize board.\n");
    // return;
    // }

    base = (volatile uint8_t*)ptr;

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
    createParam(EVENTS_STORED_STR, asynParamInt32, &eventsStoredId_);
    createParam(WINDOW_WIDTH_STR, asynParamInt32, &windowWidthId_);
    createParam(WINDOW_OFFSET_STR, asynParamInt32, &windowOffsetId_);
    createParam(SOFTWARE_CLEAR_STR, asynParamInt32, &softwareClearId_);
    createParam(SOFTWARE_TRIGGER_STR, asynParamInt32, &softwareTriggerId_);
    createParam(TDC_HEADER_TRAILER_STR, asynParamInt32, &tdcHeaderTrailerId_);
    createParam(TESTREG_STR, asynParamInt32, &testregId_);
    createParam(DUMMY16_STR, asynParamInt32, &dummy16Id_);
    createParam(DUMMY32_STR, asynParamInt32, &dummy32Id_);
    createParam(DEV_PARAM_STR, asynParamInt32, &devParamId_);

    epicsThreadCreate("CaenV1290NPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

bool CaenV1290N::wait_micro_handshake(uint16_t mask, uint16_t timeout) {
    while (true) {
        uint16_t hs;
        if (!readD16(Register::MicroHandshake, hs)) {
            return false;
        }
        if ((hs & mask) == 0 && (timeout > 0)) {
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
        if (!write_micro(Opcode::WriteEnablePattern, value)) {
            asyn_status = asynError;
        }
    } else if (function == controlId_) {
        if (!writeD16(Register::Control, value)) {
            asyn_status = asynError;
        }
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
        if (!read_micro(Opcode::ReadEnablePattern, v16)) {
            asyn_status = asynError;
        }
        *value = v16;
    } else if (function == controlId_) {
        if (!readD16(Register::Control, v16)) {
            asyn_status = asynError;
        }
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
        if (!read_micro(Opcode::ReadEdgeDetectionMode, v16)) {
            asyn_status = asynError;
        };
        *value = v16;
    } else if (function == acquisitionModeId_) {
        if (!read_micro(Opcode::ReadAcquisitionMode, v16)) {
            asyn_status = asynError;
        };
        *value = v16;
    } else if (function == tdcHeaderTrailerId_) {
        if (!read_micro(Opcode::TDCHeaderTrailerStatus, v16)) {
            asyn_status = asynError;
        };
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
        if (!write_micro(Opcode::SetEdgeDetectionMode, value)) {
            asyn_status = asynError;
        }
    } else if (function == acquisitionModeId_) {
        if (!write_micro(value == 0 ? Opcode::SetContinuous : Opcode::SetTriggerMatch)) {
            asyn_status = asynError;
        }
    } else if (function == windowWidthId_) {
        if (!write_micro(Opcode::SetWindowWidth, value)) {
            asyn_status = asynError;
        }
    } else if (function == windowOffsetId_) {
        if (!write_micro(Opcode::SetWindowOffset, value)) {
            asyn_status = asynError;
        }
    } else if (function == tdcHeaderTrailerId_) {
        if (!write_micro(value == 0 ? Opcode::DisableTDCHeaderTrailer : Opcode::EnableTDCHeaderTrailer)) {
            asyn_status = asynError;
        }
    } else if (function == softwareClearId_) {
        printf("Writing %d to software clear register\n", value);
        // this "works"
        // *(volatile epicsUInt16*)(base+Register::SwClear) = value;
        //
        // // this "works" but returns an error code
        // // NOTE: need to compile with vxLib.h, etc.
        // long status = vx_writeD16(Register::SwClear, value);
        // printf("vx_writeD16, status = %ld\n", status);
        //
        // this "works" but returns an error code
        if (!writeD16(Register::SwClear, value)) {
            printf("Write to software clear register failed\n");
            asyn_status = asynError;
        }
    } else if (function == softwareTriggerId_) {
        if (!writeD16(Register::SwTrigger, value)) {
            printf("Write to software trigger register failed\n");
            asyn_status = asynError;
        }
    } else if (function == dummy16Id_) {
        if (!writeD16(Register::Dummy16, value)) {
            printf("Write to dummy16 register failed\n");
            asyn_status = asynError;
        } else {
            printf("Wrote %d to dummy16 register\n", value);
        }
    } else if (function == dummy32Id_) {
        if (!writeD32(Register::Dummy32, value)) {
            printf("Write to dummy32 register failed\n");
            asyn_status = asynError;
        } else {
            printf("Wrote %d to dummy32 register\n", value);
        }
    } else if (function == testregId_) {
        if (writeD32(Register::TestReg, value)) {
            printf("Wrote 0x%X to test register\n", value);
        } else {
            printf("Error writing 0x%X to test register\n", value);
            asyn_status = asynError;
        }
    }

    // Output Buffer test using TEST_FIFO mode (see manual section 6.22)
    // When TEST_FIFO is enabled in the Control Register, writing a D32 word
    // to the Testreg (0x1028) loads it into the Output Buffer for readback.
    else if (function == devParamId_) {
        const uint32_t testPatterns[] = {0xDEADBEEF, 0x12345678, 0x00000000, 0xFFFFFFFF};
        const int numPatterns = sizeof(testPatterns) / sizeof(testPatterns[0]);
        int passed = 0;

        // 1. Read current Control register value
        uint16_t savedControl = 0;
        if (!readD16(Register::Control, savedControl)) {
            printf("TEST_FIFO: failed to read Control register\n");
            asyn_status = asynError;
            goto test_done;
        }

        // 2. Enable TEST_FIFO bit in Control register
        // NOTE: writing to Control also triggers a module CLEAR (manual Table 4.2),
        // which clears the output buffer for us, so no separate SwClear needed.
        if (!writeD16(Register::Control, savedControl | Control::TestFifoEnable)) {
            printf("TEST_FIFO: failed to enable TEST_FIFO in Control register\n");
            asyn_status = asynError;
            goto test_restore;
        }
        printf("TEST_FIFO: enabled (Control = 0x%04X)\n", savedControl | Control::TestFifoEnable);

        // Allow board to settle after CLEAR triggered by Control write
        epicsThreadSleep(0.1);

        // Diagnostic: verify Control readback confirms TEST_FIFO is set
        {
            uint16_t controlReadback = 0;
            readD16(Register::Control, controlReadback);
            printf("TEST_FIFO: Control readback = 0x%04X (TEST_FIFO bit %s)\n",
                   controlReadback,
                   (controlReadback & Control::TestFifoEnable) ? "SET" : "NOT SET");
        }

        // Diagnostic: verify Testreg is accessible
        {
            uint32_t diagVal = 0;
            bool ok = readD32(Register::TestReg, diagVal);
            printf("TEST_FIFO: Testreg read %s, value = 0x%08X\n", ok ? "OK" : "FAIL", diagVal);
        }

        // Diagnostic: check Status and EventStored before test
        {
            uint16_t status = 0, evStored = 0;
            readD16(Register::Status, status);
            readD16(Register::EventStored, evStored);
            printf("TEST_FIFO: pre-test Status = 0x%04X (DATA_READY=%d), EventStored = %d\n",
                   status, !!(status & Status::DataReady), evStored);
        }

        // Diagnostic: probe multiple addresses in the Output Buffer range
        printf("TEST_FIFO: probing Output Buffer address range...\n");
        {
            const uint32_t probeAddrs[] = {0x0000, 0x0004, 0x0008, 0x0100, 0x0800, 0x0FFC};
            for (int i = 0; i < (int)(sizeof(probeAddrs)/sizeof(probeAddrs[0])); i++) {
                uint32_t val = 0xCAFECAFE;
                bool ok = readD32(probeAddrs[i], val);
                printf("  base+0x%04X: %s, value = 0x%08X\n",
                       probeAddrs[i], ok ? "OK" : "FAIL", val);
            }
        }

        // Diagnostic: also try D16 reads from the Output Buffer range
        printf("TEST_FIFO: trying D16 reads from Output Buffer range...\n");
        {
            uint16_t val = 0xCAFE;
            bool ok = readD16(0x0000, val);
            printf("  base+0x0000 D16: %s, value = 0x%04X\n", ok ? "OK" : "FAIL", val);
            val = 0xCAFE;
            ok = readD16(0x0002, val);
            printf("  base+0x0002 D16: %s, value = 0x%04X\n", ok ? "OK" : "FAIL", val);
        }

        // Write one test pattern to Testreg
        printf("TEST_FIFO: writing 0xDEADBEEF to Testreg...\n");
        writeD32(Register::TestReg, 0xDEADBEEF);

        // Verify write and check status
        {
            uint32_t tregReadback = 0;
            readD32(Register::TestReg, tregReadback);
            uint16_t status = 0;
            readD16(Register::Status, status);
            printf("TEST_FIFO: Testreg=0x%08X, Status=0x%04X (DR=%d)\n",
                   tregReadback, status, !!(status & Status::DataReady));
        }

        // Try Output Buffer read
        {
            uint32_t readback = 0xCAFECAFE;
            bool ok = readD32(Register::OutBuf, readback);
            printf("TEST_FIFO: OutBuf D32 read %s, value = 0x%08X\n",
                   ok ? "OK" : "FAIL", readback);
        }

        // Now try with BERR_EN set — might change Output Buffer read behavior
        printf("TEST_FIFO: retrying with BERR_EN set...\n");
        writeD16(Register::Control, savedControl | Control::TestFifoEnable | Control::BerrEn);
        {
            uint16_t controlReadback = 0;
            readD16(Register::Control, controlReadback);
            printf("TEST_FIFO: Control = 0x%04X\n", controlReadback);
        }

        // Write another pattern
        writeD32(Register::TestReg, 0x12345678);
        {
            uint32_t tregReadback = 0;
            readD32(Register::TestReg, tregReadback);
            uint16_t status = 0;
            readD16(Register::Status, status);
            printf("TEST_FIFO: Testreg=0x%08X, Status=0x%04X (DR=%d)\n",
                   tregReadback, status, !!(status & Status::DataReady));
        }

        // Try Output Buffer read with BERR_EN
        {
            uint32_t readback = 0xCAFECAFE;
            bool ok = readD32(Register::OutBuf, readback);
            printf("TEST_FIFO: OutBuf D32 read (BERR_EN) %s, value = 0x%08X\n",
                   ok ? "OK" : "FAIL", readback);
        }

        // Print the local (CPU) address for reference
        printf("TEST_FIFO: base ptr = %p, OutBuf addr = %p, Testreg addr = %p\n",
               (void*)base, (void*)(base + Register::OutBuf), (void*)(base + Register::TestReg));

        printf("TEST_FIFO: %d/%d patterns passed\n", passed, numPatterns);

test_restore:
        // 5. Restore original Control register value (clears TEST_FIFO bit)
        writeD16(Register::Control, savedControl);
        printf("TEST_FIFO: restored Control register to 0x%04X\n", savedControl);
test_done:
        ;
    }

    if (asyn_status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readInt32\n");
    }

    return asynSuccess;
}

void CaenV1290N::poll() {
    while (true) {
        uint16_t val16 = 0;
        uint32_t val32 = 0;

        lock();

        // Read status register
        val16 = 0;
        if (readD16(Register::Status, val16)) {
            setUIntDigitalParam(statusId_, val16, 0xFFFF);
        }

        // read dummy registers for testing
        val16 = 0;
        if (!readD16(Register::Dummy16, val16)) {
            printf("Failure reading dummy16\n");
        }
        setIntegerParam(dummy16Id_, val16);

        val32 = 0;
        if (!readD32(Register::Dummy32, val32)) {
            printf("Failure reading dummy32\n");
        }
        setIntegerParam(dummy32Id_, val32);

        val16 = 0;
        if (!readD16(Register::EventStored, val16)) {
            printf("Failure reading EventStored\n");
        }
        setIntegerParam(eventsStoredId_, val16);

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
