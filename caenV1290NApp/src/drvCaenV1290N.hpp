#pragma once
#include <asynPortDriver.h>
#include <epicsMMIO.h>
#include <stdint.h>

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
    virtual asynStatus readInt32(asynUser* pasynUser, epicsInt32* value);
    virtual asynStatus readUInt32Digital(asynUser* pasynUser, epicsUInt32* value, epicsUInt32 mask);
    virtual asynStatus writeUInt32Digital(asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask);

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
        if (!wait_micro_handshake(Handshake::WriteOk)) {
            printf("Timeout waiting for micro handshake write\n");
            return false;
        }
        writeD16(Register::Micro, opcode);

        if (!wait_micro_handshake(Handshake::WriteOk)) {
            printf("Timeout waiting for micro handshake write\n");
            return false;
        }
        writeD16(Register::Micro, val);
        return true;
    }

    bool write_micro(uint16_t opcode) {
        if (!wait_micro_handshake(Handshake::WriteOk)) {
            printf("Timeout waiting for micro handshake write\n");
            return false;
        }
        writeD16(Register::Micro, opcode);
        return true;
    }

    template <typename T>
    bool read_micro(uint16_t opcode, T& retval) {
        if (!wait_micro_handshake(Handshake::WriteOk)) {
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
