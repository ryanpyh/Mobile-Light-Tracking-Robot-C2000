//#############################################################################
//
// FILE:   F28379dCAN.c

//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "F28x_Project.h"
#include "F28379dCAN.h"
//
// Defines
//
//For standard CAN Message ID is 18, for extend CAN Message ID ID shift is 29
//
#define CAN_MSG_ID_SHIFT        18U

#define CAN_MAX_BIT_DIVISOR     (13)   // The maximum CAN bit timing divisor
#define CAN_MIN_BIT_DIVISOR     (5)    // The minimum CAN bit timing divisor
#define CAN_MAX_PRE_DIVISOR     (1024) // The maximum CAN pre-divisor
#define CAN_MIN_PRE_DIVISOR     (1)    // The minimum CAN pre-divisor

//
// Globals
//


uint16_t txMsgData[8];
uint16_t rxMsgData[8];
uint32_t txMsgSize = sizeof(txMsgData);


void InitCANA(void)
{
    int16_t iMsg;

    //
    // Place CAN controller in init state, regardless of previous state.  This
    // will put controller in idle, and allow the message object RAM to be
    // programmed.
    //
    CanaRegs.CAN_CTL.bit.Init = 1;
    CanaRegs.CAN_CTL.bit.SWR = 1;

    //
    // Wait for busy bit to clear
    //
    while(CanaRegs.CAN_IF1CMD.bit.Busy)
    {
    }

    //
    // Clear the message value bit in the arbitration register.  This indicates
    // the message is not valid and is a "safe" condition to leave the message
    // object.  The same arb reg is used to program all the message objects.
    //
    CanaRegs.CAN_IF1CMD.bit.DIR = 1;
    CanaRegs.CAN_IF1CMD.bit.Arb = 1;
    CanaRegs.CAN_IF1CMD.bit.Control = 1;

    CanaRegs.CAN_IF1ARB.all = 0;

    CanaRegs.CAN_IF1MCTL.all = 0;

    CanaRegs.CAN_IF2CMD.bit.DIR = 1;
    CanaRegs.CAN_IF2CMD.bit.Arb = 1;
    CanaRegs.CAN_IF2CMD.bit.Control = 1;

    CanaRegs.CAN_IF2ARB.all = 0;

    CanaRegs.CAN_IF2MCTL.all = 0;

    //
    // Loop through to program all 32 message objects
    //
    for(iMsg = 1; iMsg <= 32; iMsg+=2)
    {
        //
        // Wait for busy bit to clear
        //
        while(CanaRegs.CAN_IF1CMD.bit.Busy)
        {
        }

        //
        // Initiate programming the message object
        //
        CanaRegs.CAN_IF1CMD.bit.MSG_NUM = iMsg;

        //
        // Wait for busy bit to clear
        //
        while(CanaRegs.CAN_IF2CMD.bit.Busy)
        {
        }

        //
        // Initiate programming the message object
        //
        CanaRegs.CAN_IF2CMD.bit.MSG_NUM = iMsg + 1;
    }

    //
    // Acknowledge any pending status interrupts.
    //
    volatile uint32_t discardRead = CanaRegs.CAN_ES.all;

}

void InitCANB(void)
{
    int16_t iMsg;

    //
    // Place CAN controller in init state, regardless of previous state.  This
    // will put controller in idle, and allow the message object RAM to be
    // programmed.
    //
    CanbRegs.CAN_CTL.bit.Init = 1;
    CanbRegs.CAN_CTL.bit.SWR = 1;

    //
    // Wait for busy bit to clear
    //
    while(CanbRegs.CAN_IF1CMD.bit.Busy)
    {
    }

    //
    // Clear the message value bit in the arbitration register.  This indicates
    // the message is not valid and is a "safe" condition to leave the message
    // object.  The same arb reg is used to program all the message objects.
    //
    CanbRegs.CAN_IF1CMD.bit.DIR = 1;
    CanbRegs.CAN_IF1CMD.bit.Arb = 1;
    CanbRegs.CAN_IF1CMD.bit.Control = 1;

    CanbRegs.CAN_IF1ARB.all = 0;

    CanbRegs.CAN_IF1MCTL.all = 0;

    CanbRegs.CAN_IF2CMD.bit.DIR = 1;
    CanbRegs.CAN_IF2CMD.bit.Arb = 1;
    CanbRegs.CAN_IF2CMD.bit.Control = 1;

    CanbRegs.CAN_IF2ARB.all = 0;

    CanbRegs.CAN_IF2MCTL.all = 0;

    //
    // Loop through to program all 32 message objects
    //
    for(iMsg = 1; iMsg <= 32; iMsg+=2)
    {
        //
        // Wait for busy bit to clear
        //
        while(CanbRegs.CAN_IF1CMD.bit.Busy)
        {
        }

        //
        // Initiate programming the message object
        //
        CanbRegs.CAN_IF1CMD.bit.MSG_NUM = iMsg;

        //
        // Wait for busy bit to clear
        //
        while(CanbRegs.CAN_IF2CMD.bit.Busy)
        {
        }

        //
        // Initiate programming the message object
        //
        CanbRegs.CAN_IF2CMD.bit.MSG_NUM = iMsg + 1;
    }

    //
    // Acknowledge any pending status interrupts.
    //
    volatile uint32_t discardRead = CanbRegs.CAN_ES.all;

}

uint32_t setCANBitRate(uint32_t sourceClock, uint32_t bitRate)
{
    uint32_t desiredRatio;
    uint32_t canBits;
    uint32_t preDivide;
    uint32_t regValue;
    uint16_t canControlValue;

    //
    // Calculate the desired clock rate.
    //
    desiredRatio = sourceClock / bitRate;

    //
    // Make sure that the Desired Ratio is not too large.  This enforces the
    // requirement that the bit rate is larger than requested.
    //
    if((sourceClock / desiredRatio) > bitRate)
    {
        desiredRatio += 1;
    }

    //
    // Check all possible values to find a matching value.
    //
    while(desiredRatio <= CAN_MAX_PRE_DIVISOR * CAN_MAX_BIT_DIVISOR)
    {
        //
        // Loop through all possible CAN bit divisors.
        //
        for(canBits = CAN_MAX_BIT_DIVISOR;
            canBits >= CAN_MIN_BIT_DIVISOR;
            canBits--)
        {
            //
            // For a given CAN bit divisor save the pre divisor.
            //
            preDivide = desiredRatio / canBits;

            //
            // If the calculated divisors match the desired clock ratio then
            // return these bit rate and set the CAN bit timing.
            //
            if((preDivide * canBits) == desiredRatio)
            {
                //
                // Start building the bit timing value by adding the bit timing
                // in time quanta.
                //
                regValue = canBitValues[canBits - CAN_MIN_BIT_DIVISOR];

                //
                // To set the bit timing register, the controller must be
                // placed
                // in init mode (if not already), and also configuration change
                // bit enabled.  The state of the register should be saved
                // so it can be restored.
                //
                canControlValue = CanbRegs.CAN_CTL.all;
                CanbRegs.CAN_CTL.bit.Init = 1;
                CanbRegs.CAN_CTL.bit.CCE = 1;

                //
                // Now add in the pre-scalar on the bit rate.
                //
                regValue |= ((preDivide - 1) & CAN_BTR_BRP_M) |
                            (((preDivide - 1) << 10) & CAN_BTR_BRPE_M);

                //
                // Set the clock bits in the and the bits of the
                // pre-scalar.
                //
                CanbRegs.CAN_BTR.all = regValue;

                //
                // Restore the saved CAN Control register.
                //
                CanbRegs.CAN_CTL.all = canControlValue;

                //
                // Return the computed bit rate.
                //
                return(sourceClock / ( preDivide * canBits));
            }
        }

        //
        // Move the divisor up one and look again.  Only in rare cases are
        // more than 2 loops required to find the value.
        //
        desiredRatio++;
    }
    return 0;
}

void CANsetupMessageObject(uint32_t base, uint32_t objID, uint32_t msgID, CAN_MsgFrameType frame, CAN_MsgObjType msgType, uint32_t msgIDMask, uint32_t flags, uint16_t msgLen)
{
    uint32_t cmdMaskReg = 0U;
    uint32_t maskReg = 0U;
    uint32_t arbReg = 0U;
    uint32_t msgCtrl = 0U;

    //
    // Check the arguments.
    //
//    ASSERT(CAN_isBaseValid(base));
//    ASSERT((objID <= 32U) && (objID > 0U));
//    ASSERT(msgLen <= 8U);

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    switch(msgType)
    {
        //
        // Transmit message object.
        //
        case CAN_MSG_OBJ_TYPE_TX:
        {
            //
            // Set message direction to transmit.
            //
            arbReg = CAN_IF1ARB_DIR;
            break;
        }

        //
        // Remote frame receive remote, with auto-transmit message object.
        //
        case CAN_MSG_OBJ_TYPE_RXTX_REMOTE:
        {
            //
            // Set message direction to Tx for remote receivers.
            //
            arbReg = CAN_IF1ARB_DIR;

            //
            // Set this object to auto answer if a matching identifier is seen.
            //
            msgCtrl = (uint32_t)((uint32_t)CAN_IF1MCTL_RMTEN |
                                 (uint32_t)CAN_IF1MCTL_UMASK);

            break;
        }

        //
        // Transmit remote request message object (CAN_MSG_OBJ_TYPE_TX_REMOTE)
        // or Receive message object (CAN_MSG_OBJ_TYPE_RX).
        //
        default:
        {
           //
           // Set message direction to read.
           //
           arbReg = 0U;

           break;
        }
    }

    //
    // Set values based on Extended Frame or Standard Frame
    //
    if(frame == CAN_MSG_FRAME_EXT)
    {
        //
        // Configure the Mask Registers for 29 bit Identifier mask.
        //
        if((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
            maskReg = msgIDMask & CAN_IF1MSK_MSK_M;
        }

        //
        // Set the 29 bit version of the Identifier for this message
        // object. Mark the message as valid and set the extended ID bit.
        //
        arbReg |= (msgID & CAN_IF1ARB_ID_M) | CAN_IF1ARB_MSGVAL |
                  CAN_IF1ARB_XTD;
    }
    else
    {
        //
        // Configure the Mask Registers for 11 bit Identifier mask.
        //
        if((flags & CAN_MSG_OBJ_USE_ID_FILTER) == CAN_MSG_OBJ_USE_ID_FILTER)
        {
           maskReg = ((msgIDMask << CAN_IF1ARB_STD_ID_S) &
                      CAN_IF1ARB_STD_ID_M);
        }

        //
        // Set the 11 bit version of the Identifier for this message
        // object. The lower 18 bits are set to zero. Mark the message as
        // valid.
        //
        arbReg |= ((msgID << CAN_IF1ARB_STD_ID_S) & CAN_IF1ARB_STD_ID_M) |
                  CAN_IF1ARB_MSGVAL;
    }

    //
    // If the caller wants to filter on the extended ID bit then set it.
    //
    maskReg |= (flags & CAN_MSG_OBJ_USE_EXT_FILTER);

    //
    // The caller wants to filter on the message direction field.
    //
    maskReg |= (flags & CAN_MSG_OBJ_USE_DIR_FILTER);

    //
    // If any filtering is requested, set the UMASK bit to use mask register
    //
    if(((flags & CAN_MSG_OBJ_USE_ID_FILTER) |
        (flags & CAN_MSG_OBJ_USE_DIR_FILTER) |
        (flags & CAN_MSG_OBJ_USE_EXT_FILTER)) != 0U)
    {
        msgCtrl |= CAN_IF1MCTL_UMASK;
    }

    //
    // Set the data length for the transfers.
    // This is applicable for Tx mailboxes.
    //
    msgCtrl |= ((uint32_t)msgLen & CAN_IF1MCTL_DLC_M);

    //
    // If this is a single transfer or the last mailbox of a FIFO, set EOB bit.
    // If this is not the last entry in a FIFO, leave the EOB bit as 0.
    //
    if((flags & CAN_MSG_OBJ_FIFO) == 0U)
    {
        msgCtrl |= CAN_IF1MCTL_EOB;
    }

    //
    // Enable transmit interrupts if they should be enabled.
    //
    msgCtrl |= (flags & CAN_MSG_OBJ_TX_INT_ENABLE);

    //
    // Enable receive interrupts if they should be enabled.
    //
    msgCtrl |= (flags & CAN_MSG_OBJ_RX_INT_ENABLE);

    //
    // Set the Control, Arb, and Mask bit so that they get transferred to the
    // Message object.
    //
    cmdMaskReg |= CAN_IF1CMD_ARB;
    cmdMaskReg |= CAN_IF1CMD_CONTROL;
    cmdMaskReg |= CAN_IF1CMD_MASK;
    cmdMaskReg |= CAN_IF1CMD_DIR;

    //
    // Write out the registers to program the message object.
    //
    HWREG_BP(base + CAN_O_IF1MSK) = maskReg;
    HWREG_BP(base + CAN_O_IF1ARB) = arbReg;
    HWREG_BP(base + CAN_O_IF1MCTL) = msgCtrl;

    //
    // Transfer data to message object RAM
    //
    HWREG_BP(base + CAN_O_IF1CMD) =
    cmdMaskReg | (objID & CAN_IF1CMD_MSG_NUM_M);
}


void CANsendMessage(uint32_t base, uint32_t objID, uint16_t msgLen, const uint16_t *msgData)
{
    uint32_t msgCtrl = 0U;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID > 0U));
    ASSERT(msgLen <= 8U);

    //
    // Set IF command to read message object control value
    //
    // Set up the request for data from the message object.
    // Transfer the message object to the IF register.
    //
    HWREG_BP(base + CAN_O_IF1CMD) = ((uint32_t)CAN_IF1CMD_CONTROL |
                                     (objID & CAN_IF1CMD_MSG_NUM_M));

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) == CAN_IF1CMD_BUSY)
    {
    }

    //
    // Read IF message control
    //
    msgCtrl = HWREGH(base + CAN_O_IF1MCTL);

    //
    // Check provided DLC size with actual Message DLC size
    //
    ASSERT((msgCtrl & CAN_IF1MCTL_DLC_M) == msgLen);

    //
    // Write the data out to the CAN Data registers.
    //
    CAN_writeDataReg(msgData, (base + CAN_O_IF1DATA),
                     (msgCtrl & CAN_IF1MCTL_DLC_M));

    //
    //  Set Data to be transferred from IF
    //
    if(msgLen > 0U)
    {
        msgCtrl = CAN_IF1CMD_DATA_B | CAN_IF1CMD_DATA_A;
    }
    else
    {
        msgCtrl = 0U;
    }

    //
    // Set Direction to write
    //
    // Set Tx Request Bit
    //
    // Transfer the message object to the message object specified by
    // objID.
    //
    HWREG_BP(base + CAN_O_IF1CMD) = (msgCtrl | (uint32_t)CAN_IF1CMD_DIR |
                                     (uint32_t)CAN_IF1CMD_TXRQST |
                                     (objID & CAN_IF1CMD_MSG_NUM_M));
}



bool CANreadMessage(uint32_t base, uint32_t objID, uint16_t *msgData)
{
    bool status;
    uint16_t msgCtrl = 0U;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID != 0U));

    //
    // Set the Message Data A, Data B, and control values to be read
    // on request for data from the message object.
    //
    // Transfer the message object to the message object IF register.
    //
    HWREG_BP(base + CAN_O_IF2CMD) =
    ((uint32_t)CAN_IF2CMD_DATA_A | (uint32_t)CAN_IF2CMD_DATA_B |
     (uint32_t)CAN_IF2CMD_CONTROL | (objID & CAN_IF2CMD_MSG_NUM_M) |
     (uint32_t)CAN_IF2CMD_ARB);

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
    {
    }

    //
    // Read out the IF control Register.
    //
    msgCtrl = HWREGH(base + CAN_O_IF2MCTL);

    //
    // See if there is new data available.
    //
    if((msgCtrl & CAN_IF2MCTL_NEWDAT) == CAN_IF2MCTL_NEWDAT)
    {
        //
        // Read out the data from the CAN registers.
        //
        CAN_readDataReg(msgData, (base + CAN_O_IF2DATA),
                        (msgCtrl & CAN_IF2MCTL_DLC_M));

        status = true;

        //
        // Now clear out the new data flag
        //
        HWREG_BP(base + CAN_O_IF2CMD) = ((uint32_t)CAN_IF2CMD_TXRQST |
                                        (objID & CAN_IF2CMD_MSG_NUM_M));

        //
        // Wait for busy bit to clear
        //
        while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) ==
               CAN_IF2CMD_BUSY)
        {
        }
    }
    else
    {
        status = false;
    }

    return(status);
}


uint32_t CANgetInterruptCause(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Read interrupt identifier status
    //
    return(HWREG_BP(base + CAN_O_INT));
}

uint16_t CANgetStatus(uint32_t base)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));

    //
    // Return error and status register value
    //
    return(HWREGH(base + CAN_O_ES));
}

void CANclearInterruptStatus(uint32_t base, uint32_t intClr)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intClr == CAN_INT_INT0ID_STATUS) ||
           ((intClr >= 1U) && (intClr <= 32U)));

    if(intClr == (uint32_t)CAN_INT_INT0ID_STATUS)
    {
        //
        // Simply read and discard the status to clear the interrupt.
        //
        HWREGH(base + CAN_O_ES);
    }
    else
    {
        //
        // Wait to be sure that this interface is not busy.
        //
        while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) ==
              CAN_IF1CMD_BUSY)
        {
        }

        //
        // Only change the interrupt pending state by setting only the
        // CAN_IF1CMD_CLRINTPND bit.
        //
        // Send the clear pending interrupt command to the CAN controller.
        //
        HWREG_BP(base + CAN_O_IF1CMD) = ((uint32_t)CAN_IF1CMD_CLRINTPND |
                                        (intClr & CAN_IF1CMD_MSG_NUM_M));

        //
        // Wait to be sure that this interface is not busy.
        //
        while((HWREGH(base + CAN_O_IF1CMD) & CAN_IF1CMD_BUSY) ==
              CAN_IF1CMD_BUSY)
        {
        }
    }
}

void CANclearGlobalInterruptStatus(uint32_t base, uint16_t intFlags)
{
    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((intFlags & ~(CAN_GLOBAL_INT_CANINT0 |
                         CAN_GLOBAL_INT_CANINT1)) == 0U);

    //
    // Clear the requested interrupts
    //
    HWREGH(base + CAN_O_GLB_INT_CLR) |= intFlags;
}

void InterruptclearACKGroup(uint16_t group)
{
    //
    // Set interrupt group acknowledge bits
    //
    HWREGH(PIECTRL_BASE + PIE_O_ACK) = group;
}

//
// End of File
//
