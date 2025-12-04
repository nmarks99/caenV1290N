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

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char *portName, int baseAddress);

    // These are the methods that we override from asynPortDriver
    // virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    // virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    // virtual asynStatus getBounds(asynUser *pasynUser, epicsInt32 *low,
    // epicsInt32 *high); virtual asynStatus writeFloat64(asynUser *pasynUser,
    // epicsFloat64 value); virtual asynStatus readFloat64(asynUser *pasynUser,
    // epicsFloat64 *value);
    // virtual void report(FILE *fp, int details);

    // protected:
    // int CAEN_V1290N_Data;          [>*< (asynInt32, r/w) discriminator value in
    // device units <] #define FIRST_PARAM CAEN_V1290N_Data int
    // CAEN_V1290N_DoubleData;    [>*< (asynFloat64, r/w) discriminator value in
    // device units but double <] #define LAST_PARAM CAEN_V1290N_DoubleData

  private:
    int lastChan;
    int maxValue;
    volatile uint32_t *regs;
};

// #define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)
#define NUM_PARAMS 0

CaenV1290N::CaenV1290N(const char *portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, asynInt32Mask | asynFloat64Mask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask, ASYN_MULTIDEVICE, ASYN_CANBLOCK, 0,
                     0) // Default priority and stack size
{
    volatile void *ptr;
    const unsigned int BASE_ADDRESS = 0xB2000000;
    const unsigned int EXTENT = 0x04000000; // 64MB window

    if (devRegisterAddress("CAEN_V1290N", atVMEA32, BASE_ADDRESS, EXTENT, &ptr)) {
        printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
        return;
    }
    char *baseBytePtr = (char *)ptr;

    const unsigned long CONTROL_OFFSET = 0x1000;
    const uint16_t TEST_FIFO_ENABLE_MASK = 0x0040; // Bit 6
    void *controlRegAddress = baseBytePtr + CONTROL_OFFSET;

    // Perform D16 write to enable TEST_FIFO (This is assumed to succeed)
    *( (volatile uint16_t *)controlRegAddress ) = TEST_FIFO_ENABLE_MASK;
    printf("Wrote 0x%04X to Control Reg (0x1000) to enable TEST_FIFO.\n", TEST_FIFO_ENABLE_MASK);

    const unsigned long TESTREG_OFFSET = 0x1028;
    const uint32_t testPattern = 0xDEADBEEF;
    void *testRegAddress = baseBytePtr + TESTREG_OFFSET;

    // Perform D32 write to the Testreg (This is confirmed not to crash, implying success)
    *( (volatile uint32_t *)testRegAddress ) = testPattern;
    printf("Wrote test pattern 0x%X to Testreg (0x1028) using D32.\n", testPattern);

    // Read the first 16 bits of the register using the D16 VME transaction
    volatile uint16_t readValue16 = *( (volatile uint16_t *)testRegAddress );

    // check for the upper 16 bits: 0xDEAD.
    const uint16_t expected16BitValue = 0xDEAD;
    printf("Attempted D16 Read from Testreg (0x1028): 0x%04X\n", readValue16);

    if (readValue16 != 0xFFFF) {
        printf("SUCCESS: The D16 read transaction completed (returned 0x%04X).\n", readValue16);
        if (readValue16 == expected16BitValue) {
            printf("RESULT: D16 read matched the MSBs of the written pattern.\n");
        } else {
             printf("RESULT: D16 read succeeded but returned an unexpected value.\n");
        }

    } else {
        printf("FAILURE: D16 read also returned 0xFFFF (VME Bus Error/Timeout).\n");
    }

}

// asynStatus CAEN_V1290N::readInt32(asynUser *pasynUser, epicsInt32 *value) {
    // int channel;
    // static const char *functionName = "readInt32";
//
    // this->getAddress(pasynUser, &channel);
    // if (channel<0 || channel>this->lastChan) {
        // return(asynError);
    // }
    // *value=this->regs[channel];
    // *value=0;
    // asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              // "%s:%s, port %s, read %d from channel %d\n",
              // driverName, functionName, this->portName, *value, channel);
    // return(asynSuccess);
// }

extern "C" int initCaenV1290N(const char *portName, int baseAddress) {
    new CaenV1290N(portName, baseAddress);
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
