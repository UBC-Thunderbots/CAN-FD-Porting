# Motor Driver — SPI to CAN FD Migration

**File:** `interface_2.c`  
**Author:** Tomas Navas  
**Status:** Initial implementation — pending compilation and hardware testing

## 1. Migration Overview

This is the first attempt at migrating the previous SPI interface to CAN FD communication. So far, the overall command-processing and telemetry-response flow has been preserved, while the SPI DMA reception and software buffering mechanisms have been replaced by the STM32's hardware FDCAN RX and TX FIFOs.

The implementation also incorporates the `FDCAN_RxHeaderTypeDef` and `FDCAN_TxHeaderTypeDef` structures defined by the STM32 HAL to handle CAN frame metadata.

For further information on these functions and structures, refer to [STMicroelectronics' FDCAN HAL documentation](https://dev.st.com/stm32cube-docs/hal1-to-hal2-migration/1.0.0/en/docs/markup/drivers_documentation/hal_drivers/fdcan/hal_fdcan_exported_functions_io_operation.html).

**Note:** Our current STM32G4 implementation uses the HAL1 API, so care should be taken to distinguish HAL1 functions from HAL2 functions when consulting this guide.

Additionally, the previous software TX queue and dequeue functions were removed. The FDCAN peripheral manages incoming messages using hardware RX FIFOs and outgoing messages using its hardware TX FIFO. Each FIFO stores messages in first-in, first-out order, although CAN bus arbitration can affect when outgoing messages are successfully transmitted.

### 1.1 Main Architectural Differences Between SPI and CAN FD

<img width="282" height="505" alt="SPI_Flowchart" src="https://github.com/user-attachments/assets/8ef3dca2-fe0c-42b0-bad1-c4f691fd7df2" />
<img width="350" height="581" alt="CAN_Flowchart" src="https://github.com/user-attachments/assets/4b47ceba-2b21-4d07-a205-3faed2d4abc5" />



## 2. CAN FD Initialization

### 2.1 FDCAN Peripheral Configuration

STM32CubeMX was used to configure the FDCAN peripheral, using the STSPIN32G4 pinout as a reference.

The FDCAN1 pins are configured as:

- **PA11:** `FDCAN1_RX`
- **PA12:** `FDCAN1_TX`

The exact pin assignments and package compatibility must be verified against the actual board schematic and generated CubeMX configuration.

### 2.2 CAN Identifiers and Acceptance Filters

`CAN_COMMAND_ID` identifies command messages intended for the motor driver, while `CAN_RESPONSE_ID` identifies outgoing telemetry responses.

The FDCAN peripheral supports hardware acceptance filters, which determine which incoming CAN identifiers are stored in the RX FIFOs. These can be configured using `HAL_FDCAN_ConfigFilter()`.

The final identifiers and hardware acceptance filter configuration still need to be confirmed with the team.

## 3. Changes to Message Reception

In the previous SPI implementation, received bytes were stored in DMA buffers and appended to a software stream using `AppendRxBytes()`. `PopNextFrame()` then extracted complete messages by checking their delimiters and CRC.

In the CAN FD implementation, message reception is handled by the hardware RX FIFOs. Since CAN FD already provides frame boundaries and hardware CRC validation, the software stream buffer and parsing functions are no longer necessary.

The current compatibility implementation still preserves the original application-level delimiters and CRC checks inside `ProcessCANFrame()`.

### 3.1 Removal of SPI DMA and Stream Buffering

The previous `HandleCompletedDmaHalf()`, `AppendRxBytes()`, and `PopNextFrame()` functions were removed from the reception process.

Their responsibilities are now handled by the FDCAN peripheral and the new `PopNextCanFrame()` function.

### 3.2 `PopNextCanFrame()` and Hardware RX FIFOs

In `PopNextCanFrame()`, the algorithm first uses `HAL_FDCAN_GetRxFifoFillLevel()` to check whether FIFO 0 contains any received messages. If FIFO 0 is empty, it checks FIFO 1.

When a message is available, `HAL_FDCAN_GetRxMessage()` retrieves it from the selected FIFO.

For future improvements, we could dedicate each FIFO to specific message types through the hardware acceptance filters.

## 4. Changes to `Interface_Loop()`

The interface loop now uses `PopNextCanFrame()` to poll for incoming CAN messages instead of checking SPI DMA completion flags.

It also activates TX-complete notifications using `HAL_FDCAN_ActivateNotification()`. The bitwise OR operator combines the TX buffer masks, while the return-value check determines whether notification setup succeeded.

When a transmission completes, the FDCAN interrupt handler can invoke `HAL_FDCAN_TxBufferCompleteCallback()` to update the transmission counter, provided the corresponding interrupts are configured.

### 4.1 CAN Header Validation and Command Processing

Inside `Interface_Loop()`, the `FDCAN_RxHeaderTypeDef` structure is used to validate incoming messages.

The firmware checks the CAN identifier, frame type, CAN FD format, and expected payload length before passing the message to `ProcessCANFrame()`.

Messages that fail these conditions are ignored.

## 5. Changes to Message Processing

### 5.1 `ProcessSpiFrame()` to `ProcessCANFrame()`

`ProcessCANFrame()` is mostly an adaptation of the previous `ProcessSpiFrame()` function.

After validating the application-level message format and sequence number, the function reads the opcode from `rx[4]` and uses the existing switch statement to call the appropriate motor-control handler.

The CAN identifier is not included in the `rx[]` payload array. It is part of the CAN header and has already been checked in `Interface_Loop()`.

After processing a valid command, the function generates the requested telemetry response and prepares it for CAN transmission.

### 5.2 Message Formatting and Transmission

For now, we are keeping the original 10-byte SPI application message inside a 12-byte CAN FD payload.

The original delimiters, sequence number, opcode, command parameters, and application-level CRC remain in their previous positions.

For transmission, the firmware creates an `FDCAN_TxHeaderTypeDef` structure and uses `HAL_FDCAN_GetTxFifoFreeLevel()` to check whether the hardware TX FIFO has space.

If space is available, `HAL_FDCAN_AddMessageToTxFifoQ()` submits the telemetry response. If the FIFO is full or submission fails, `unsent_msg_count` is incremented and the response is dropped.

## 6. Current Limitations and Next Steps

### 6.1 Compilation and Hardware Testing

The implementation is still pending compilation within the complete motor driver firmware project and testing on the actual hardware.

Testing should verify that incoming CAN FD commands are correctly received and processed, telemetry responses are transmitted, and transmission-completion notifications work as expected.

### 6.2 CAN Protocol Confirmation and Future Improvements

The final CAN command and response identifiers, message format, and acceptance filter configuration still need to be confirmed with the team.

Once communication is working, we could also consider removing the original SPI delimiters and application-level CRC, since CAN FD already provides hardware framing and CRC checking.

### 6.3 Communication Timeout and Error Handling

The current implementation does not yet provide complete handling for edge cases, such as:

- Communication timeouts
- CAN bus errors
- Dropped responses

Before motor operation, a communication timeout should be implemented and tested to ensure the motor enters a safe state if communication with the controller is lost.

Future improvements should also address bus-off recovery and TX FIFO overflow handling.
