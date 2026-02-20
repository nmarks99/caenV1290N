#pragma once
#include <asynPortDriver.h>
#include <epicsMMIO.h>
#include <devLib.h>
#include <stdint.h>

// #include "V1290N.hpp"

// String names for asyn parameters
#define ACQUISITION_MODE_STR "ACQUISITION_MODE"
#define EDGE_DETECT_MODE_STR "EDGE_DETECT_MODE"
#define ENABLE_PATTERN_STR "ENABLE_PATTERN"
#define WINDOW_WIDTH_STR "WINDOW_WIDTH"
#define WINDOW_OFFSET_STR "WINDOW_OFFSET"
#define SOFTWARE_CLEAR_STR "SOFTWARE_CLEAR"
#define STATUS_STR "STATUS"
#define CONTROL_STR "CONTROL"
#define TESTREG_STR "TESTREG"
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
    volatile uint8_t* base;

    /// \brief Continually tests microcontroller handshake until true, or timeout.
    ///
    /// \param mask The mask to test handshake register with.
    /// \param timeout Timeout in milliseconds.
    /// \return True on success, false on error or timeout/
    bool wait_micro_handshake(uint16_t mask, uint16_t timeout=1000);

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
	return !devWriteProbe(sizeof(uint16_t), base+offset, &value);
    }

    /// \brief Performs a safe D16 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool readD16(uint16_t offset, uint16_t& value) {
	return !devReadProbe(sizeof(uint16_t), base+offset, &value);
    }

    /// \brief Performs a safe D32 VME bus write of the value to the offset
    /// \param offset The offset from the base address to write to
    /// \param value The value to write
    /// \return True on success, false on error
    bool writeD32(uint32_t offset, uint32_t value) {
	return !devWriteProbe(sizeof(uint32_t), base+offset, &value);
    }

    /// \brief Performs a safe D32 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool readD32(uint32_t offset, uint32_t& value) {
	return !devReadProbe(sizeof(uint32_t), base+offset, &value);
    }

  protected:
    int acquisitionModeId_;
    int edgeDetectModeId_;
    int enablePatternId_;
    int windowWidthId_;
    int windowOffsetId_;
    int softwareClearId_;
    int statusId_;
    int controlId_;
    int testregId_;
    int devParamId_;
};
