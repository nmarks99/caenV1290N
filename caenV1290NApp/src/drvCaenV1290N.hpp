#pragma once
#include <asynPortDriver.h>
#include <devLib.h>
#include <epicsMMIO.h>
#include <stdint.h>

// #warning "vxWorks dependent for testing"
// #include <vxWorks.h>
// #include <vme.h>
// #include <vxLib.h>

// String names for asyn parameters
#define ACQUISITION_MODE_STR "ACQUISITION_MODE"
#define EDGE_DETECT_MODE_STR "EDGE_DETECT_MODE"
#define ENABLE_PATTERN_STR "ENABLE_PATTERN"
#define WINDOW_WIDTH_STR "WINDOW_WIDTH"
#define WINDOW_OFFSET_STR "WINDOW_OFFSET"
#define SOFTWARE_CLEAR_STR "SOFTWARE_CLEAR"
#define SOFTWARE_TRIGGER_STR "SOFTWARE_TRIGGER"
#define TDC_HEADER_TRAILER_STR "TDC_HEADER_TRAILER"
#define STATUS_STR "STATUS"
#define CONTROL_STR "CONTROL"
#define TESTREG_STR "TESTREG"
#define DUMMY32_STR "DUMMY32"
#define DUMMY16_STR "DUMMY16"
#define EVENTS_STORED_STR "EVENTS_STORED"
#define DEV_PARAM_STR "DEV_PARAM"

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual void poll();
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser* pasynUser, epicsInt32* value);
    virtual asynStatus readUInt32Digital(asynUser* pasynUser, epicsUInt32* value, epicsUInt32 mask);
    virtual asynStatus writeUInt32Digital(asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask);

  private:
    // this is a "trick" since adding an offset like 0x1000 to a pointer to uint8_t moves 4 bytes
    volatile uint8_t* base;

    /// \brief Continually tests microcontroller handshake until true, or timeout.
    ///
    /// \param mask The mask to test handshake register with.
    /// \param timeout Timeout in milliseconds.
    /// \return True on success, false on error or timeout/
    bool wait_micro_handshake(uint16_t mask, uint16_t timeout = 1000);

    /// \brief Writes given opcode, followed by given value to the micro register.
    ///
    /// \param opcode The opcode to write.
    /// \param val The value to write immediately after the opcode.
    /// \return True on success, false on error or timeout.
    bool write_micro(uint16_t opcode, uint16_t val);

    /// \brief Writes given opcode to the micro register.
    ///
    /// \param opcode The opcode to write.
    /// \return True on success, false on error or timeout.
    bool write_micro(uint16_t opcode);

    /// \brief Reads data stored in micro register after writing given opcode
    ///
    /// \param opcode The opcode to write.
    /// \param value Reference to the value to store the data in.
    /// \return True on success, false on error or timeout.
    bool read_micro(uint16_t opcode, uint16_t& value);

    /// \brief Performs a safe D16 VME bus write of the value to the offset
    /// \param offset The offset from the base address to write to
    /// \param value The value to write
    /// \return True on success, false on error
    bool writeD16(uint16_t offset, uint16_t value) {
        return !devWriteProbe(sizeof(uint16_t), base + offset, &value);
    }

    /// \brief Performs a safe D16 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool readD16(uint16_t offset, uint16_t& value) {
        return !devReadProbe(sizeof(uint16_t), base + offset, &value);
    }

    /// \brief Performs a safe D32 VME bus write of the value to the offset
    /// \param offset The offset from the base address to write to
    /// \param value The value to write
    /// \return True on success, false on error
    bool writeD32(uint32_t offset, uint32_t value) {
        return !devWriteProbe(sizeof(uint32_t), base + offset, &value);
    }

    /// \brief Performs a safe D32 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool readD32(uint32_t offset, uint32_t& value) {
        return !devReadProbe(sizeof(uint32_t), base + offset, &value);
    }

    // // devWriteProbe is the OSI abstraction on top of vxMemProbe, so as expected,
    // // this appears to give the same result as CaenV1290N::writeD16
    // bool writeD32(uint32_t offset, uint32_t value) {
    // printf("vxMemProbe write called\n");
    // return !vxMemProbe( (char*)(base+offset), VX_WRITE, sizeof(uint32_t), (char*)(&value));
    // }

    // static long vxDevReadProbe (unsigned wordSize, volatile const void *ptr, void *pValue)
    // {
    // long status;
    //
    // status = vxMemProbe ((char *)ptr, VX_READ, wordSize, (char *) pValue);
    // if (status!=OK) {
    // return S_dev_noDevice;
    // }
    //
    // return 0;
    // }
    //
    // static long vxDevWriteProbe (unsigned wordSize, volatile void *ptr, const void *pValue)
    // {
    // long status;
    //
    // status = vxMemProbe ((char *)ptr, VX_WRITE, wordSize, (char *) pValue);
    // if (status!=OK) {
    // return S_dev_noDevice;
    // }
    //
    // return 0;
    // }

  protected:
    int acquisitionModeId_;
    int edgeDetectModeId_;
    int enablePatternId_;
    int windowWidthId_;
    int windowOffsetId_;
    int softwareClearId_;
    int softwareTriggerId_;
    int tdcHeaderTrailerId_;
    int statusId_;
    int eventsStoredId_;
    int controlId_;
    int testregId_;
    int dummy16Id_;
    int dummy32Id_;
    int devParamId_;
};
