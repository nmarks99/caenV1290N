
#include <stdlib.h>

#include <epicsTypes.h>
#include <errMdef.h>
#include <dbDefs.h>
#include <devLib.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynPortDriver.h>

#define MAX_CHANNELS 16

static const char *driverName = "CAEN_V1290N";

// Register Offsets
#define V1290_FIRMWARE_REV  0x1026  // D16
#define V1290_SW_CLEAR      0x1016  // D16, Write-only

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char *portName, int baseAddress);
  private:
    volatile unsigned short *regs;
};

// #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)
#define NUM_PARAMS 0

CaenV1290N::CaenV1290N(const char *portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, asynInt32Mask | asynFloat64Mask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask, ASYN_MULTIDEVICE, ASYN_CANBLOCK, 0,
                     0) // Default priority and stack size
{

    volatile void *ptr;
    const unsigned int EXTENT = 0x04000000; // 64MB window
    if (devRegisterAddress("CAEN_V1290N", atVMEA32, baseAddress, EXTENT, &ptr)) {
	printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
	return;
    }
    regs = (volatile unsigned short*)ptr;
    printf("Success?\n");

    // char *vmeAddress;
    // // 1. Map the VME address to local memory.
    // // Use VME_AM_STD_SUP_DATA (0x3D) for A24 or VME_AM_EXT_SUP_DATA (0x09) for A32.
    // if (sysBusToLocalAdrs(VME_AM_STD_SUP_DATA, (char*)baseAddress, &vmeAddress) != OK) {
        // printf("Error: Could not map VME address 0x%08X\n", baseAddress);
        // return;
    // }
    // regs = (volatile unsigned short *)vmeAddress;

    // 2. Optional: Initialize/Clear the board
    // Writing any value to the Software Clear Register resets logic and clears buffers.
    regs[V1290_SW_CLEAR / 2] = 1;
    printf("Board logic cleared.\n");

    // 3. Read the Firmware Revision Register
    // Note: Offset is divided by 2 because 'regs' is a 16-bit pointer.
    unsigned short revValue = regs[V1290_FIRMWARE_REV / 2];

    // 4. Decode the revision components
    int major = (revValue >> 4) & 0x0F;
    int minor = revValue & 0x0F;

    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n", major, minor, revValue);

}

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
