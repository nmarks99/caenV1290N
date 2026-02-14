#pragma once

#include <epicsTypes.h>

#define MAX_CHANNELS 16

namespace Register {
// Data Buffer
static const epicsUInt16 OutBuf = 0x0000; // D32, R, Output Buffer

// Configuration & Status Registers
static const epicsUInt16 Control = 0x1000;         // D16, R/W, Control Register
static const epicsUInt16 Status = 0x1002;          // D16, R, Status Register
static const epicsUInt16 IntLevel = 0x100A;        // D16, R/W, epicsUInt16errupt Level
static const epicsUInt16 IntVector = 0x100C;       // D16, R/W, epicsUInt16errupt Vector
static const epicsUInt16 GeoAddr = 0x100E;         // D16, R/W, GEO Address
static const epicsUInt16 McstCbltAddr = 0x1010;    // D16, R/W, MCST/CBLT Address
static const epicsUInt16 McstCbltCtrl = 0x1012;    // D16, R/W, MCST/CBLT Control
static const epicsUInt16 ModReset = 0x1014;        // D16, W, Module Reset
static const epicsUInt16 SwClear = 0x1016;         // D16, W, Software Clear
static const epicsUInt16 SwEventReset = 0x1018;    // D16, W, Software Event Reset
static const epicsUInt16 SwTrigger = 0x101A;       // D16, W, Software Trigger
static const epicsUInt16 EventCounter = 0x101C;    // D32, R, Event Counter
static const epicsUInt16 EventStored = 0x1020;     // D16, R, Event Stored
static const epicsUInt16 AlmostFullLevel = 0x1022; // D16, R/W, Almost Full Level
static const epicsUInt16 BltEventNumber = 0x1024;  // D16, R/W, BLT Event Number
static const epicsUInt16 FirmwareRev = 0x1026;     // D16, R, Firmware Revision
static const epicsUInt16 TestReg = 0x1028;         // D32, R/W, Test Register
static const epicsUInt16 OutProgControl = 0x102C;  // D16, R/W, Output Prog. Control
static const epicsUInt16 Micro = 0x102E;           // D16, R/W, Micro Register (Opcodes)
static const epicsUInt16 MicroHandshake = 0x1030;  // D16, R, Micro Handshake
static const epicsUInt16 SelectFlash = 0x1032;     // D16, R/W, Select Flash
static const epicsUInt16 FlashRW = 0x1034;         // D16, R/W, Flash R/W
static const epicsUInt16 SramPage = 0x1036;        // D16, R/W, SRAM Page
static const epicsUInt16 EventFifo = 0x1038;       // D32, R, Event FIFO
static const epicsUInt16 EventFifoStored = 0x103C; // D16, R, Event FIFO Stored
static const epicsUInt16 EventFifoStatus = 0x103E; // D16, R, Event FIFO Status
static const epicsUInt16 Dummy32 = 0x1200;         // D32, R/W, Dummy 32-bit
static const epicsUInt16 Dummy16 = 0x1204;         // D16, R/W, Dummy 16-bit
} // namespace Reg

namespace Control {
static const epicsUInt16 BerrEn = (1 << 0);         // Bus Error enable for BLT
static const epicsUInt16 Term = (1 << 1);           // Software termination status
static const epicsUInt16 TermSw = (1 << 2);         // 1=SW Term, 0=HW Term
static const epicsUInt16 EmptyEvent = (1 << 3);     // Enable Header/Trailer for empty events
static const epicsUInt16 Align64 = (1 << 4);        // 64-bit alignment (dummy word)
static const epicsUInt16 CompEnable = (1 << 5);     // INL compensation enable
static const epicsUInt16 TestFifoEnable = (1 << 6); // Output Buffer test mode
static const epicsUInt16 ReadCompEnable = (1 << 7); // Read compensation SRAM
static const epicsUInt16 EventFifoEn = (1 << 8);    // Event FIFO enable
static const epicsUInt16 EtttEnable = (1 << 9);     // Extended Trigger Time Tag enable
static const epicsUInt16 Bus16mbAddrEn = (1 << 10); // 16MB address range for MEB
} // namespace Control

namespace Status {
static const epicsUInt16 DataReady = (1 << 0);    // Data/Event ready in Buffer
static const epicsUInt16 AlmostFull = (1 << 1);   // Almost Full level reached
static const epicsUInt16 Full = (1 << 2);         // Output Buffer full
static const epicsUInt16 TrgMatchMode = (1 << 3); // 1=Trigger Match, 0=Continuous
static const epicsUInt16 HeaderEn = (1 << 4);     // TDC Header/Trailer enabled
static const epicsUInt16 TermOn = (1 << 5);       // Termination status
static const epicsUInt16 Error0 = (1 << 6);       // TDC 0 Error
static const epicsUInt16 Error1 = (1 << 7);       // TDC 1 Error
static const epicsUInt16 Error2 = (1 << 8);       // TDC 2 Error (V1290A only)
static const epicsUInt16 Error3 = (1 << 9);       // TDC 3 Error (V1290A only)
static const epicsUInt16 BerrFlag = (1 << 10);    // VME Bus Error occurred
static const epicsUInt16 Purged = (1 << 11);      // Board purged
static const epicsUInt16 TriggerLost = (1 << 15); // Trigger overlap/lost
} // namespace Status

namespace Handshake {
static const epicsUInt16 WriteOk = (1 << 0); // Ready for Micro write/opcode
static const epicsUInt16 ReadOk = (1 << 1);  // Data ready for Micro read
} // namespace Handshake

namespace Opcode {
static const epicsUInt16 SetTriggerMatch = 0x0000;
static const epicsUInt16 SetContinuous = 0x0100;
static const epicsUInt16 ReadAcqMode = 0x0200;
} // namespace Opcode
