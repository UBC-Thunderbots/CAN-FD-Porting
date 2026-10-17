/**
 ******************************************************************************
 * @file    interface.c
 * @author  UBC Thunderbots
 * @brief   First attempt at the CAN port of the previous interface
 *
* This version uses the STM32G4 FDCAN hardware RX/TX FIFOs and preserves
 * the original command/telemetry encoding. Not hardware-tested. 
 * Application message layout (10 bytes, carried in 12-byte CAN FD frame):
 *
 *  [0]    0xAA  delimiter byte 0
 *  [1]    0xBB  delimiter byte 1
 *  [2]    0xCC  delimiter byte 2
 *  [3]    SEQ   sequence number
 *  [4]    OP    opcode (RX) or response type (TX)
 *  [5..8] DATA  payload (4 bytes)
 *  [9]    CRC   checksum over bytes [0..8]
 *
 ******************************************************************************
 */

#include "interface_2.h"

#include "stm32g4xx_hal.h"
#include "motorcontrol.h"
#include "speed_feed_forward_ctrl.h"
#include "crc.h"

#include <stdbool.h>
#include <string.h>

/* CAN_COMMAND_ID and CAN_RESPONSE_ID must be defined in interface_2.h. */
#ifndef CAN_COMMAND_ID
#error "Define team-approved CAN_COMMAND_ID in interface_2.h"
#endif
#ifndef CAN_RESPONSE_ID
#error "Define team-approved CAN_RESPONSE_ID in interface_2.h"
#endif


/* External variables --------------------------------------------------------*/

extern FDCAN_HandleTypeDef hfdcan1;

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;

/* Protocol format ---------------------------------------------------------*/

#if MESSAGE_SIZE != 10U
#error "This compatibility port requires the original 10-byte MESSAGE_SIZE"
#endif

/* Diagnostics: TX submissions rejected and CAN transmissions completed. */
volatile uint32_t unsent_msg_count = 0U;
volatile uint32_t can_tx_count = 0U;

/* Private variables ---------------------------------------------------------*/

ResponseType_t response_type;

bool motor_enabled;

uint8_t last_seq_num;

/* Private function prototypes -----------------------------------------------*/

static void PopulateTxHeader(uint8_t *tx, uint8_t seq, ResponseType_t type);
static void PopulateTx_ResponseForCurrentType(uint8_t *tx, uint8_t seq);


void ProcessRx_SetTargetSpeed(const uint8_t *rx);
void ProcessRx_SetTargetTorque(const uint8_t *rx);
void ProcessRx_SetResponseType(const uint8_t *rx);
void ProcessRx_SetPidTorqueKpKi(const uint8_t *rx);
void ProcessRx_SetPidFluxKpKi(const uint8_t *rx);
void ProcessRx_SetPidSpeedKpKi(const uint8_t *rx);
void ProcessRx_SetSpeedFeedForwardKaKv(const uint8_t *rx);
void ProcessRx_SetSpeedFeedForwardKs(const uint8_t *rx);

void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIqAndId(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseVqAndVd(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx, uint8_t seq);
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx, uint8_t seq);

void SetMotorEnabled(bool enabled);

//New functions I implemented on 10/11-14
static bool PopNextCanFrame(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
static void ProcessCANFrame(const uint8_t *rx);

/* Function implementations --------------------------------------------------*/

/**
 * Initializes the CAN interface module.
 */
void Interface_Init(void)
{
  unsent_msg_count = 0U;
  can_tx_count = 0U;
  response_type = SPEED_AND_FAULTS;
  motor_enabled = false;
  last_seq_num = 0;
}


/**
 * Main loop for the CAN interface module (polling RX FIFOs).
 */
void Interface_Loop(void)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_TX_COMPLETE,
                                   FDCAN_TX_BUFFER0 |
                                   FDCAN_TX_BUFFER1 |
                                   FDCAN_TX_BUFFER2) != HAL_OK)
  {
    return;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return;
  }

  while (1)
  {
    if (!PopNextCanFrame(&rx_header, rx_data))
    {
      continue;
    }

    if (rx_header.IdType != FDCAN_STANDARD_ID ||
        rx_header.Identifier != CAN_COMMAND_ID ||
        rx_header.RxFrameType != FDCAN_DATA_FRAME ||
        rx_header.FDFormat != FDCAN_FD_CAN ||
        rx_header.DataLength != FDCAN_DLC_BYTES_12)
    {
      continue;
    }

    ProcessCANFrame(rx_data);
  }
}

/**
 * Retrieves one complete frame from the configured FDCAN RX FIFOs.
 */
static bool PopNextCanFrame(FDCAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
  uint32_t RX_FIFO_to_use;

  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
  {
    RX_FIFO_to_use = FDCAN_RX_FIFO0;
  }
  else if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) > 0U)
  {
    RX_FIFO_to_use = FDCAN_RX_FIFO1;
  }
  else
  {
    return false;
  }

  return HAL_FDCAN_GetRxMessage(&hfdcan1, RX_FIFO_to_use, rxHeader, rxData) == HAL_OK;
}


/**
 * Populates the common prefix for a transmit frame.
 */
static void PopulateTxHeader(uint8_t *tx, uint8_t seq, ResponseType_t type)
{
  memset(tx, 0, MESSAGE_SIZE);
  tx[0] = MESSAGE_DELIMITER_0;
  tx[1] = MESSAGE_DELIMITER_1;
  tx[2] = MESSAGE_DELIMITER_2;
  tx[3] = seq;
  tx[4] = (uint8_t)type;
}

/**
 * Populates a response frame using the currently selected response type.
 */
static void PopulateTx_ResponseForCurrentType(uint8_t *tx, uint8_t seq)
{

  /* Response type selects telemetry contents; CAN ID is in txHeader. */
  switch (response_type) 
  {
    case SPEED_AND_FAULTS:
      PopulateTx_ResponseSpeedAndFaults(tx, seq);
      break;
    case IQ_AND_ID:
      PopulateTx_ResponseIqAndId(tx, seq);
      break;
    case VQ_AND_VD:
      PopulateTx_ResponseVqAndVd(tx, seq);
      break;
    case PHASE_CURRENT_AND_VOLTAGE:
      PopulateTx_ResponsePhaseCurrentAndVoltage(tx, seq);
      break;
    case IQ_AND_IQ_REF:
      PopulateTx_ResponseIqAndIqRef(tx, seq);
      break;
    case ID_AND_ID_REF:
      PopulateTx_ResponseIdAndIdRef(tx, seq);
      break;
    case SPEED_AND_SPEED_REF:
      PopulateTx_ResponseSpeedAndSpeedRef(tx, seq);
      break;
  }
}

/**
 * Processes a CAN FD payload containing the original 10-byte command.
 * Generates and submits matching telemetry via the hardware TX FIFO.
 */
static void ProcessCANFrame(const uint8_t *rx)
{
 
  if (rx[0] != MESSAGE_DELIMITER_0 ||
      rx[1] != MESSAGE_DELIMITER_1 ||
      rx[2] != MESSAGE_DELIMITER_2 ||
      rx[9] != CrcGenerateChecksum(rx, MESSAGE_SIZE - 1))
  {
    return;
  }

  const uint8_t seq = rx[3];
  const uint8_t expected_same = last_seq_num;
  const uint8_t expected_next = (uint8_t)(last_seq_num + 1U);
  if (seq != expected_same && seq != expected_next)
  {
    return;
  }

  switch ((Opcode_t)rx[4])
  {
    case NO_OP: break;
    case SET_TARGET_SPEED: ProcessRx_SetTargetSpeed(rx); break;
    case SET_TARGET_TORQUE: ProcessRx_SetTargetTorque(rx); break;
    case SET_RESPONSE_TYPE: ProcessRx_SetResponseType(rx); break;
    case SET_PID_TORQUE_KP_KI: ProcessRx_SetPidTorqueKpKi(rx); break;
    case SET_PID_FLUX_KP_KI: ProcessRx_SetPidFluxKpKi(rx); break;
    case SET_PID_SPEED_KP_KI: ProcessRx_SetPidSpeedKpKi(rx); break;
    case SET_SPEED_FEED_FORWARD_KA_KV: ProcessRx_SetSpeedFeedForwardKaKv(rx); break;
    case SET_SPEED_FEED_FORWARD_KS: ProcessRx_SetSpeedFeedForwardKs(rx); break;
    default: return;
  }
  last_seq_num = seq;

  uint8_t tx[12] = {0};

PopulateTx_ResponseForCurrentType(tx, seq);

FDCAN_TxHeaderTypeDef txHeader = {0};

// The header holds CAN arbitration/format metadata; tx holds telemetry data.

  txHeader.Identifier = CAN_RESPONSE_ID;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_12;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_ON;
  txHeader.FDFormat = FDCAN_FD_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0U;


 //Checking wether there's space in the queue before sending message 
if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ( &hfdcan1, &txHeader, tx) != HAL_OK)
    {
        unsent_msg_count++;
    }
}
else
{
    unsent_msg_count++;
}
}





/**
 * Processes the received SPI frame to update the target speed for
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+---------------+-------+
 *  | Enable |     Speed     |       |
 *  +--------+---------------+-------+
 *      1        2       3       4
 */
void ProcessRx_SetTargetSpeed(const uint8_t *rx)
{
  const int16_t motor_target_speed = (int16_t)(((uint16_t)rx[6] << 8) | rx[7]);
  MC_ProgramSpeedRampMotor1(motor_target_speed, 0);
  SetMotorEnabled(rx[5] != 0);
}

/**
 * Processes the received SPI frame to update the target torque for
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+---------------+-------+
 *  | Enable |    Torque     |       |
 *  +--------+---------------+-------+
 *      1        2       3       4
 */
void ProcessRx_SetTargetTorque(const uint8_t *rx)
{
  const int16_t motor_target_torque = (int16_t)(((uint16_t)rx[6] << 8) | rx[7]);
  MC_ProgramTorqueRampMotor1(motor_target_torque, 0);
  SetMotorEnabled(rx[5] != 0);
}

/**
 * Processes the received SPI frame to update the type of response
 * that will be sent back to the master in subsequent transactions.
 *
 *  +--------+-----------------------+
 *  |  Type  |                       |
 *  +--------+-----------------------+
 *      1         2      3      4
 */
void ProcessRx_SetResponseType(const uint8_t *rx)
{
  response_type = (ResponseType_t)rx[5];
}

/**
 * Processes the received SPI frame to update the Kp and Ki
 * parameters for the torque (Iq) PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4
 */
void ProcessRx_SetPidTorqueKpKi(const uint8_t *rx)
{
  const int16_t kp = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  const int16_t ki = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

  PID_SetKP(&PIDIqHandle_M1, kp);
  PID_SetKI(&PIDIqHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki
 * parameters for the flux (Id) PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4
 */
void ProcessRx_SetPidFluxKpKi(const uint8_t *rx)
{
  const int16_t kp = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  const int16_t ki = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

  PID_SetKP(&PIDIdHandle_M1, kp);
  PID_SetKI(&PIDIdHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki
 * parameters for the speed PID controller.
 *
 *  +----------------+---------------+
 *  |       Kp       |       Ki      |
 *  +----------------+---------------+
 *      1       2        3       4
 */
void ProcessRx_SetPidSpeedKpKi(const uint8_t *rx)
{
  const int16_t kp = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  const int16_t ki = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

  PID_SetKP(&PIDSpeedHandle_M1, kp);
  PID_SetKI(&PIDSpeedHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Ka and Kv
 * parameters for the speed feedforward controller.
 *
 *  +----------------+---------------+
 *  |       Ka       |       Kv      |
 *  +----------------+---------------+
 *      1       2        3       4
 */
void ProcessRx_SetSpeedFeedForwardKaKv(const uint8_t *rx)
{
  const int16_t ka = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);
  const int16_t kv = (int16_t)(((uint16_t)rx[7] << 8) | rx[8]);

  SpeedFF_SetKaGain(&SpeedFF_M1, (int32_t)ka);
  SpeedFF_SetKvGain(&SpeedFF_M1, (int32_t)kv);
}

/**
 * Processes the received SPI frame to update the Ks
 * parameter for the speed feedforward controller.
 *
 *  +----------------+---------------+
 *  |       Ks       |               |
 *  +----------------+---------------+
 *      1       2        3       4
 */
void ProcessRx_SetSpeedFeedForwardKs(const uint8_t *rx)
{
  const int16_t ks = (int16_t)(((uint16_t)rx[5] << 8) | rx[6]);

  SpeedFF_SetKsGain(&SpeedFF_M1, (int32_t)ks);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and motor faults.
 *
 *  +---------------+----------------+
 *  |  Speed (RPM)  |     Faults     |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx, uint8_t seq)
{
  const int16_t motor_current_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_faults = MC_GetOccurredFaultsMotor1();

  PopulateTxHeader(tx, seq, SPEED_AND_FAULTS);
  tx[5] = (uint8_t)((motor_current_speed >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_current_speed & 0xFF);
  tx[7] = (uint8_t)((motor_faults >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_faults & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and Id values for the motor.
 *
 *  +---------------+----------------+
 *  |      Iq       |       Id       |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseIqAndId(uint8_t *tx, uint8_t seq)
{
  const qd_t motor_iqd = MC_GetIqdMotor1();

  PopulateTxHeader(tx, seq, IQ_AND_ID);
  tx[5] = (uint8_t)((motor_iqd.q >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd.q & 0xFF);
  tx[7] = (uint8_t)((motor_iqd.d >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_iqd.d & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Vq and Vd values for the motor.
 *
 *  +---------------+----------------+
 *  |      Vq       |       Vd       |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseVqAndVd(uint8_t *tx, uint8_t seq)
{
  const qd_t motor_vqd = MC_GetVqdMotor1();

  PopulateTxHeader(tx, seq, VQ_AND_VD);
  tx[5] = (uint8_t)((motor_vqd.q >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_vqd.q & 0xFF);
  tx[7] = (uint8_t)((motor_vqd.d >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_vqd.d & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * phase current and voltage amplitudes for the motor.
 *
 *  +---------------+----------------+
 *  | Phase Current |  Phase Voltage |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx, uint8_t seq)
{
  const int16_t motor_phase_current = MC_GetPhaseCurrentAmplitudeMotor1();
  const int16_t motor_phase_voltage = MC_GetPhaseVoltageAmplitudeMotor1();

  PopulateTxHeader(tx, seq, PHASE_CURRENT_AND_VOLTAGE);
  tx[5] = (uint8_t)((motor_phase_current >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_phase_current & 0xFF);
  tx[7] = (uint8_t)((motor_phase_voltage >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_phase_voltage & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and reference Iq values for the motor.
 *
 *  +---------------+----------------+
 *  |      Iq       |     Iq Ref     |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx, uint8_t seq)
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  PopulateTxHeader(tx, seq, IQ_AND_IQ_REF);
  tx[5] = (uint8_t)((motor_iqd.q >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd.q & 0xFF);
  tx[7] = (uint8_t)((motor_iqd_ref.q >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_iqd_ref.q & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Id and reference Id values for the motor.
 *
 *  +---------------+----------------+
 *  |      Id       |     Id Ref     |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx, uint8_t seq)
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  PopulateTxHeader(tx, seq, ID_AND_ID_REF);
  tx[5] = (uint8_t)((motor_iqd.d >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_iqd.d & 0xFF);
  tx[7] = (uint8_t)((motor_iqd_ref.d >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_iqd_ref.d & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and reference speed in RPM.
 *
 *  +---------------+----------------+
 *  |     Speed     |    Speed Ref   |
 *  +---------------+----------------+
 *      1       2        3      4
 */
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx, uint8_t seq)
{
  const int16_t motor_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_speed_ref = MC_GetMecSpeedReferenceMotor1();

  PopulateTxHeader(tx, seq, SPEED_AND_SPEED_REF);
  tx[5] = (uint8_t)((motor_speed >> 8) & 0xFF);
  tx[6] = (uint8_t)(motor_speed & 0xFF);
  tx[7] = (uint8_t)((motor_speed_ref >> 8) & 0xFF);
  tx[8] = (uint8_t)(motor_speed_ref & 0xFF);
  tx[9] = CrcGenerateChecksum(tx, MESSAGE_SIZE - 1);
}

/**
 * Enables or disables the motor based on the provided flag.
 *
 * @param enabled Whether to enable or disable the motor.
 */
void SetMotorEnabled(bool enabled)
{
  if (enabled && !motor_enabled)
  {
    MC_StartMotor1();
  }
  else if (!enabled && motor_enabled)
  {
    MC_StopMotor1();
  }
  motor_enabled = enabled;
}
/* Count successful CAN transmissions, not just TX FIFO submissions.
 * Requires the FDCAN TX-complete interrupt routed through HAL_FDCAN_IRQHandler().
 */
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
                                        uint32_t BufferIndexes)
{
  if (hfdcan->Instance == FDCAN1)
  {
    if ((BufferIndexes & FDCAN_TX_BUFFER0) != 0U) can_tx_count++;
    if ((BufferIndexes & FDCAN_TX_BUFFER1) != 0U) can_tx_count++;
    if ((BufferIndexes & FDCAN_TX_BUFFER2) != 0U) can_tx_count++;
  }
}


 