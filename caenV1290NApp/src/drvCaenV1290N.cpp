#include <stdlib.h>

#include <epicsTypes.h>
#include <errMdef.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <epicsMMIO.h>

#include <asynPortDriver.h>

#ifdef __rtems__
#include <rtems.h>
#include <bsp/vmeTsi148.h>
#endif

#define MAX_CHANNELS 16

// static const char *driverName = "CAEN_V1290N";

// Register Offsets
#define V1290_FIRMWARE_REV  0x1026 // D16
#define V1290_SW_CLEAR      0x1016 // D16
#define V1290_OUT_BUF       0x0000 // D32
#define V1290_CONTROL       0x1000 // D16
#define V1290_TESTREG       0x1028 // D32
#define V1290_SW_TRIGGER    0x101A // D16
#define V1290_DUMMY32       0x1200 // D32
#define V1290_DUMMY16       0x1204 // D16

#define TEST_FIFO_ENABLE 0b01000000

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char *portName, int baseAddress);
    // virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  private:
    volatile uint8_t *base;

    inline void writeD16(uint16_t offset, uint16_t value) {
	*reinterpret_cast<volatile uint16_t*>(base+offset) = value;
	#if defined(__PPC__) || defined(__powerpc__)
	    __asm__ volatile("eieio");
	#endif
    }

    inline uint16_t readD16(uint16_t offset) {
	uint16_t val = *reinterpret_cast<volatile uint16_t*>(base+offset);
	#if defined(__PPC__) || defined(__powerpc__)
	    __asm__ volatile("eieio");
	#endif
	return val;
    }

    inline void writeD32(uint32_t offset, uint32_t value) {
	*reinterpret_cast<volatile uint32_t*>(base+offset) = value;
	#if defined(__PPC__) || defined(__powerpc__)
	    __asm__ volatile("eieio");
	#endif
    }

    inline uint32_t readD32(uint32_t offset) {
	uint32_t val = *reinterpret_cast<volatile uint32_t*>(base+offset);
	#if defined(__PPC__) || defined(__powerpc__)
	    __asm__ volatile("eieio");
	#endif
	return val;
    }
};

// #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)
#define NUM_PARAMS 0

#define A32_PORT 0
#define A32_AM 0x0D
#define A32_BASE 0xB2000000
#define A32_SIZE 0x04000000

CaenV1290N::CaenV1290N(const char *portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, asynInt32Mask | asynFloat64Mask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask, ASYN_MULTIDEVICE, ASYN_CANBLOCK, 0,
                     0) // Default priority and stack size
{

#ifdef __rtems__
    // initialize A32 window???
    int status = vmeTsi148OutboundPortCfg(
	A32_PORT,
	A32_AM,
	A32_BASE,
	0x80000000,
	A32_SIZE
    );
    if (status != 0) {
	printf("Failed to configure A32 port\n");
	return;
    }
    printf("A32 window configured successfully\n");
#endif

    // initialize
    volatile void *ptr;
    // const unsigned int EXTENT = 0x04000000; // 64MB window
    const unsigned int EXTENT = 0x10000;
    if (devRegisterAddress("CAEN_V1290N", atVMEA32, baseAddress, EXTENT, &ptr)) {
	printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
	return;
    }

    base = (volatile uint8_t*)ptr;

    // Optional Safety: Check if the board is actually there before doing heavy logic
    // devReadProbe returns 0 on success (board found), -1 on failure
    uint16_t probeVal;
    if (devReadProbe(2, (volatile char *)(base + V1290_FIRMWARE_REV), (char *)&probeVal) != 0) {
        printf("ERROR: VME Bus Error - Board not found at address 0x%X\n", baseAddress);
        return;
    }

    // Writing any value to the Software Clear Register resets logic and clears buffers.
    writeD16(V1290_SW_CLEAR, 1);
    printf("Board logic cleared.\n\n");

    // Read the Firmware Revision Register
    uint16_t rev = readD16(V1290_FIRMWARE_REV);
    int major = (rev >> 4) & 0x0F;
    int minor = rev & 0x0F;
    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n\n", major, minor, rev);

    // =====================================================================

    // Enable test mode
    printf("Reading control register...\n");
    uint16_t ctrl_reg = readD16(V1290_CONTROL);
    printf("Control register = %X\n", ctrl_reg);
    printf("Setting TEST_FIFO_ENABLE bit in control register\n\n");
    ctrl_reg |= TEST_FIFO_ENABLE;
    writeD16(V1290_CONTROL, ctrl_reg);

    // write test data into test registers
    printf("Writing %X into test register...\n", 0xDEADBEEF);
    writeD32(V1290_TESTREG, 0xDEADBEEF);
    printf("Read test reg = %X\n\n", readD32(V1290_TESTREG));

    printf("Writing %X into Dummy32 register...\n", 0xCAFEBABE);
    writeD32(V1290_DUMMY32, 0xCAFEBABE);
    printf("Read Dummy32 reg = %X\n\n", readD32(V1290_DUMMY32));

    printf("Writing %X into Dummy16 register...\n", 0xCAFE);
    writeD16(V1290_DUMMY16, 0xCAFE);
    printf("Read Dummy16 reg = %X\n\n", readD16(V1290_DUMMY16));

    // Generate a software trigger
    printf("Generating software trigger...\n");
    writeD16(V1290_SW_TRIGGER, 1);

}

// asynStatus CaenV1290N::writeInt32(asynUser *pasynUser, epicsInt32 value);

extern "C" int initCaenV1290N(const char *portName, int baseAddress) {
    new CaenV1290N("CAENV1290N_TEST", 0xB2000000); // hack
    return (asynSuccess);
}

static const iocshArg initArg0 = {"Port name", iocshArgString};
static const iocshArg initArg1 = {"Base Address", iocshArgInt};
static const iocshArg *const initArgs[2] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"initCAEN_V1290N", 2, initArgs};
static void initCallFunc(const iocshArgBuf *args) { initCaenV1290N(args[0].sval, args[1].ival); }

void drvCaenV1290NRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

extern "C" {
epicsExportRegistrar(drvCaenV1290NRegister);
}
