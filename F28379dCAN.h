#ifndef F28379DCAN_H
#define F28379DCAN_H

typedef enum
{
        //! Transmit message object.
        MSG_OBJ_TYPE_TRANSMIT,

        //! Receive message object.
        MSG_OBJ_TYPE_RECEIVE
}
msgObjType;

static const uint16_t canBitValues[] =
{
    0x1100, // TSEG2 2, TSEG1 2, SJW 1, Divide 5
    0x1200, // TSEG2 2, TSEG1 3, SJW 1, Divide 6
    0x2240, // TSEG2 3, TSEG1 3, SJW 2, Divide 7
    0x2340, // TSEG2 3, TSEG1 4, SJW 2, Divide 8
    0x3340, // TSEG2 4, TSEG1 4, SJW 2, Divide 9
    0x3440, // TSEG2 4, TSEG1 5, SJW 2, Divide 10
    0x3540, // TSEG2 4, TSEG1 6, SJW 2, Divide 11
    0x3640, // TSEG2 4, TSEG1 7, SJW 2, Divide 12
    0x3740  // TSEG2 4, TSEG1 8, SJW 2, Divide 13
};

#define CANA_BASE    0x00048000U // CAN-A Registers
#define CANB_BASE    0x0004A000U // CAN-B Registers
#define CAN_INT_INT0ID_STATUS           (0x8000U)

void InitCANA(void);

void InitCANB(void);

uint32_t setCANBitRate(uint32_t sourceClock, uint32_t bitRate);

void CANsetupMessageObject(uint32_t base, uint32_t objID, uint32_t msgID, CAN_MsgFrameType frame, CAN_MsgObjType msgType, uint32_t msgIDMask, uint32_t flags, uint16_t msgLen);

void CANsendMessage(uint32_t base, uint32_t objID, uint16_t msgLen, const uint16_t *msgData);

bool CANreadMessage(uint32_t base, uint32_t objID, uint16_t *msgData);

uint32_t CANgetInterruptCause(uint32_t base);

uint16_t CANgetStatus(uint32_t base);

void CANclearInterruptStatus(uint32_t base, uint32_t intClr);

void CANclearGlobalInterruptStatus(uint32_t base, uint16_t intFlags);

void InterruptclearACKGroup(uint16_t group);

#endif
