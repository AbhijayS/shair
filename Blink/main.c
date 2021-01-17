/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

// This sample C application for Azure Sphere uses the Azure Sphere I2C APIs to display
// data from an accelerometer connected via I2C.
//
// It uses the APIs for the following Azure Sphere application libraries:
// - log (displays messages in the Device Output window during debugging)
// - i2c (communicates with LSM6DS3 accelerometer)
// - eventloop (system invokes handlers for timer events)

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/eventloop.h>
#include <applibs/uart.h>

// The following #include imports a "sample appliance" definition. This app comes with multiple
// implementations of the sample appliance, each in a separate directory, which allow the code to
// run on different hardware.
//
// By default, this app targets hardware that follows the MT3620 Reference Development Board (RDB)
// specification, such as the MT3620 Dev Kit from Seeed Studio.
//
// To target different hardware, you'll need to update CMakeLists.txt. For example, to target the
// Avnet MT3620 Starter Kit, change the TARGET_HARDWARE variable to
// "avnet_mt3620_sk".
//
// See https://aka.ms/AzureSphereHardwareDefinitions for more details.
#include <hw/template_appliance.h>

#include "eventloop_timer_utilities.h"

#define UPDATE_PERIOD_MS 10
#define BILLION 1000000

/// <summary>
/// Exit codes for this application. These are used for the
/// application exit code. They must all be between zero and 255,
/// where zero is reserved for successful termination.
/// </summary>
typedef enum {
    ExitCode_Success = 0,

    ExitCode_TermHandler_SigTerm = 1,

    ExitCode_AccelTimer_Consume = 2,
    ExitCode_AccelTimer_ReadStatus = 3,
    ExitCode_AccelTimer_ReadZAccel = 4,

    ExitCode_ReadWhoAmI_WriteThenRead = 5,
    ExitCode_ReadWhoAmI_WriteThenReadCompare = 6,
    ExitCode_ReadWhoAmI_Write = 7,
    ExitCode_ReadWhoAmI_Read = 8,
    ExitCode_ReadWhoAmI_WriteReadCompare = 9,
    ExitCode_ReadWhoAmI_PosixWrite = 10,
    ExitCode_ReadWhoAmI_PosixRead = 11,
    ExitCode_ReadWhoAmI_PosixCompare = 12,

    ExitCode_SampleRange_Reset = 13,
    ExitCode_SampleRange_SetRange = 14,

    ExitCode_Init_EventLoop = 15,
    ExitCode_Init_AccelTimer = 16,
    ExitCode_Init_OpenMaster = 17,
    ExitCode_Init_SetBusSpeed = 18,
    ExitCode_Init_SetTimeout = 19,
    ExitCode_Init_SetDefaultTarget = 20,

    ExitCode_SendMessage_Write = 22,
    ExitCode_UartEvent_Read = 23,
    ExitCode_Init_UartOpen = 24,
    ExitCode_Init_RegisterIo = 25,

    ExitCode_Main_EventLoopFail = 21
} ExitCode;

// Support functions.
static void TerminationHandler(int signalNumber);
static void AccelTimerEventHandler(EventLoopTimer *timer);
static ExitCode ReadWhoAmI(void);
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes);
static ExitCode ResetAndSetSampleRange(void);
static ExitCode InitPeripheralsAndHandlers(void);
static void CloseFdAndPrintError(int fd, const char *fdName);
static void ClosePeripheralsAndHandlers(void);
static void UartEventHandler(EventLoop *el, int fd, EventLoop_IoEvents events, void *context);
static void SendUartMessage(int uartFd, const char *dataToSend);

// File descriptors - initialized to invalid value
static int i2cFd = -1;
static int uartFd = -1;

static EventLoop *eventLoop = NULL;
static EventLoopTimer *accelTimer = NULL;
EventRegistration *uartEventReg = NULL;

// DocID026899 Rev 10, S6.1.1, I2C operation
// SDO is tied to ground so the least significant bit of the address is zero.
static const uint8_t lsm6ds3Address = 0x6A;

// Termination state
static volatile sig_atomic_t exitCode = ExitCode_Success;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    exitCode = ExitCode_TermHandler_SigTerm;
}

/// <summary>
///     Print latest data from accelerometer.
/// </summary>
static void AccelTimerEventHandler(EventLoopTimer *timer)
{
    static int iter = 1;

    if (ConsumeEventLoopTimerEvent(timer) != 0) {
        exitCode = ExitCode_AccelTimer_Consume;
        return;
    }

    // Status register describes whether accelerometer is available.
    // DocID026899 Rev 10, S9.26, STATUS_REG (1Eh); [0] = XLDA
    static const uint8_t statusRegId = 0x1E;
    uint8_t status;
    ssize_t transferredBytes = I2CMaster_WriteThenRead(
        i2cFd, lsm6ds3Address, &statusRegId, sizeof(statusRegId), &status, sizeof(status));
    if (!CheckTransferSize("I2CMaster_WriteThenRead (STATUS_REG)",
                           sizeof(statusRegId) + sizeof(status), transferredBytes)) {
        exitCode = ExitCode_AccelTimer_ReadStatus;
        return;
    }

    if ((status & 0x1) == 0) {
        Log_Debug("INFO: %d: No accel data.\n", iter);
    } else if ((status & 0x2) == 0) {
        Log_Debug("INFO: %d: No gyro data.\n", iter);
    } else {
        static const uint8_t outXG = 0x22;
        static const uint8_t outYG = 0x24;
        static const uint8_t outZG = 0x26;
        static const uint8_t outXLXl = 0x28;
        static const uint8_t outYLXl = 0x2A;
        static const uint8_t outZLXl = 0x2C;
        int16_t gxRaw;
        int16_t gyRaw;
        int16_t gzRaw;
        int16_t axRaw;
        int16_t ayRaw;
        int16_t azRaw;
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outXG, sizeof(outXG),
                                                    (uint8_t *)&gxRaw, sizeof(gxRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTX_G)",
                                sizeof(outXG) + sizeof(gxRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outYG, sizeof(outYG),
                                                    (uint8_t *)&gyRaw, sizeof(gyRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTY_G)",
                                sizeof(outYG) + sizeof(gyRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outZG, sizeof(outZG),
                                                    (uint8_t *)&gzRaw, sizeof(gzRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTZ_G)",
                                sizeof(outZG) + sizeof(gzRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outXLXl, sizeof(outXLXl),
                                                    (uint8_t *)&axRaw, sizeof(axRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTX_L_XL)",
                                sizeof(outXLXl) + sizeof(axRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outYLXl, sizeof(outYLXl),
                                                    (uint8_t *)&ayRaw, sizeof(ayRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTY_L_XL)",
                                sizeof(outYLXl) + sizeof(ayRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &outZLXl, sizeof(outZLXl),
                                                    (uint8_t *)&azRaw, sizeof(azRaw));
        if (!CheckTransferSize("I2CMaster_WriteThenRead (OUTZ_L_XL)",
                                sizeof(outZLXl) + sizeof(azRaw), transferredBytes)) {
            exitCode = ExitCode_AccelTimer_ReadZAccel;
            return;
        }

        // DocID026899 Rev 10, S4.1, Mechanical characteristics
        // These constants are specific to LA_So where FS = +/-4g, as set in CTRL1_X.
        double g = (azRaw * 0.122) / 1000.0;
        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);
        long ms = t.tv_nsec/1000000;
        // Log_Debug("INFO: %ld: vertical acceleration: %.2lfg\n", ms, g);
        char m[80];
        sprintf(m, "%ld,%d,%d,%d,%d,%d,%d\n",
        ms,
        axRaw, ayRaw, azRaw,
        gxRaw, gyRaw, gzRaw);
        SendUartMessage(uartFd, m);
    }

    ++iter;
}

/// <summary>
///     Demonstrates three ways of reading data from the attached device.
//      This also works as a smoke test to ensure the Azure Sphere device can talk to
///     the I2C device.
/// </summary>
/// <returns>
///     ExitCode_Success on success; otherwise another ExitCode value which indicates
///     the specific failure.
/// </returns>
static ExitCode ReadWhoAmI(void)
{
    // DocID026899 Rev 10, S9.11, WHO_AM_I (0Fh); has fixed value 0x69.
    static const uint8_t whoAmIRegId = 0x0F;
    static const uint8_t expectedWhoAmI = 0x6C;
    uint8_t actualWhoAmI;

    // Read register value using AppLibs combination read and write API.
    ssize_t transferredBytes =
        I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &whoAmIRegId, sizeof(whoAmIRegId),
                                &actualWhoAmI, sizeof(actualWhoAmI));
    if (!CheckTransferSize("I2CMaster_WriteThenRead (WHO_AM_I)",
                           sizeof(whoAmIRegId) + sizeof(actualWhoAmI), transferredBytes)) {
        return ExitCode_ReadWhoAmI_WriteThenRead;
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x (I2CMaster_WriteThenRead)\n", actualWhoAmI);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
        return ExitCode_ReadWhoAmI_WriteThenReadCompare;
    }

    // Read register value using AppLibs separate read and write APIs.
    transferredBytes = I2CMaster_Write(i2cFd, lsm6ds3Address, &whoAmIRegId, sizeof(whoAmIRegId));
    if (!CheckTransferSize("I2CMaster_Write (WHO_AM_I)", sizeof(whoAmIRegId), transferredBytes)) {
        return ExitCode_ReadWhoAmI_Write;
    }
    transferredBytes = I2CMaster_Read(i2cFd, lsm6ds3Address, &actualWhoAmI, sizeof(actualWhoAmI));
    if (!CheckTransferSize("I2CMaster_Read (WHO_AM_I)", sizeof(actualWhoAmI), transferredBytes)) {
        return ExitCode_ReadWhoAmI_Read;
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x (I2CMaster_Write + I2CMaster_Read)\n", actualWhoAmI);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
        return ExitCode_ReadWhoAmI_WriteReadCompare;
    }

    // Read register value using POSIX APIs.
    // This uses the I2C target address which was set earlier with
    // I2CMaster_SetDefaultTargetAddress.
    transferredBytes = write(i2cFd, &whoAmIRegId, sizeof(whoAmIRegId));
    if (!CheckTransferSize("write (WHO_AM_I)", sizeof(whoAmIRegId), transferredBytes)) {
        return ExitCode_ReadWhoAmI_PosixWrite;
    }
    transferredBytes = read(i2cFd, &actualWhoAmI, sizeof(actualWhoAmI));
    if (!CheckTransferSize("read (WHO_AM_I)", sizeof(actualWhoAmI), transferredBytes)) {
        return ExitCode_ReadWhoAmI_PosixRead;
    }
    Log_Debug("INFO: WHO_AM_I=0x%02x (POSIX read + write)\n", actualWhoAmI);
    if (actualWhoAmI != expectedWhoAmI) {
        Log_Debug("ERROR: Unexpected WHO_AM_I value.\n");
        return ExitCode_ReadWhoAmI_PosixCompare;
    }

    return ExitCode_Success;
}

/// <summary>
///    Checks the number of transferred bytes for I2C functions and prints an error
///    message if the functions failed or if the number of bytes is different than
///    expected number of bytes to be transferred.
/// </summary>
/// <returns>true on success, or false on failure</returns>
static bool CheckTransferSize(const char *desc, size_t expectedBytes, ssize_t actualBytes)
{
    if (actualBytes < 0) {
        Log_Debug("ERROR: %s: errno=%d (%s)\n", desc, errno, strerror(errno));
        return false;
    }

    if (actualBytes != (ssize_t)expectedBytes) {
        Log_Debug("ERROR: %s: transferred %zd bytes; expected %zd\n", desc, actualBytes,
                  expectedBytes);
        return false;
    }

    return true;
}

/// <summary>
///     Resets the accelerometer and sets the sample range.
/// </summary>
/// <returns>
///     ExitCode_Success on success; otherwise another ExitCode value which indicates
///     the specific failure.
/// </returns>
static ExitCode ResetAndSetSampleRange(void)
{
    // Reset device to put registers into default state.
    // DocID026899 Rev 10, S9.14, CTRL3_C (12h); [0] = SW_RESET
    static const uint8_t ctrl3cRegId = 0x12;
    const uint8_t resetCommand[] = {ctrl3cRegId, 0x01};
    ssize_t transferredBytes =
        I2CMaster_Write(i2cFd, lsm6ds3Address, resetCommand, sizeof(resetCommand));
    if (!CheckTransferSize("I2CMaster_Write (CTRL3_C)", sizeof(resetCommand), transferredBytes)) {
        return ExitCode_SampleRange_Reset;
    }

    // Wait for device to come out of reset.
    uint8_t ctrl3c;
    do {
        transferredBytes = I2CMaster_WriteThenRead(i2cFd, lsm6ds3Address, &ctrl3cRegId,
                                                   sizeof(ctrl3cRegId), &ctrl3c, sizeof(ctrl3c));
    } while (!(transferredBytes == (sizeof(ctrl3cRegId) + sizeof(ctrl3c)) && (ctrl3c & 0x1) == 0));

    // accel
    // Use sample range +/- 2g, with 1.66kHz frequency.
    // DocID026899 Rev 10, S9.12, CTRL1_XL (10h)
    static const uint8_t setCtrl1XlCommand[] = {0x10, 0x80};
    transferredBytes =
        I2CMaster_Write(i2cFd, lsm6ds3Address, setCtrl1XlCommand, sizeof(setCtrl1XlCommand));
    if (!CheckTransferSize("I2CMaster_Write (CTRL1_XL)", sizeof(setCtrl1XlCommand),
                           transferredBytes)) {
        return ExitCode_SampleRange_SetRange;
    }

    // gyro
    // Use sample range 2000 dps, with 1.66kHz frequency.
    // DocID026899 Rev 10, S9.12, CTRL2_G (11h)
    static const uint8_t setCtrl2GCommand[] = {0x11, 0x8C};
    transferredBytes =
        I2CMaster_Write(i2cFd, lsm6ds3Address, setCtrl2GCommand, sizeof(setCtrl2GCommand));
    if (!CheckTransferSize("I2CMaster_Write (CTRL2_G)", sizeof(setCtrl2GCommand),
                           transferredBytes)) {
        return ExitCode_SampleRange_SetRange;
    }

    return ExitCode_Success;
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>
///     ExitCode_Success if all resources were allocated successfully; otherwise another
///     ExitCode value which indicates the specific failure.
/// </returns>
static ExitCode InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    eventLoop = EventLoop_Create();
    if (eventLoop == NULL) {
        Log_Debug("Could not create event loop.\n");
        return ExitCode_Init_EventLoop;
    }

    // accel setup
    static const struct timespec i2cReadPeriod = {.tv_sec = 0, .tv_nsec = UPDATE_PERIOD_MS*BILLION};
    accelTimer = CreateEventLoopPeriodicTimer(eventLoop, &AccelTimerEventHandler, &i2cReadPeriod);
    if (accelTimer == NULL) {
        return ExitCode_Init_AccelTimer;
    }

    i2cFd = I2CMaster_Open(TEMPLATE_LSM6DS3_I2C);
    if (i2cFd == -1) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_OpenMaster;
    }

    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetBusSpeed;
    }

    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
        return ExitCode_Init_SetTimeout;
    }

    // This default address is used for POSIX read and write calls.  The AppLibs APIs take a target
    // address argument for each read or write.
    result = I2CMaster_SetDefaultTargetAddress(i2cFd, lsm6ds3Address);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetDefaultTargetAddress: errno=%d (%s)\n", errno,
                  strerror(errno));
        return ExitCode_Init_SetDefaultTarget;
    }

    ExitCode localExitCode = ReadWhoAmI();
    if (localExitCode != ExitCode_Success) {
        return localExitCode;
    }

    localExitCode = ResetAndSetSampleRange();
    if (localExitCode != ExitCode_Success) {
        return localExitCode;
    }
    
    // Create a UART_Config object, open the UART and set up UART event handler
    UART_Config uartConfig;
    UART_InitConfig(&uartConfig);
    uartConfig.baudRate = 115200;
    uartConfig.flowControl = UART_FlowControl_None;
    uartConfig.dataBits = UART_DataBits_Eight;
    uartConfig.parity = UART_Parity_None;
    uartConfig.stopBits = UART_StopBits_One;
    uartFd = UART_Open(TEMPLATE_UART, &uartConfig);
    if (uartFd == -1) {
        Log_Debug("ERROR: Could not open UART: %s (%d).\n", strerror(errno), errno);
        return ExitCode_Init_UartOpen;
    }
    uartEventReg = EventLoop_RegisterIo(eventLoop, uartFd, EventLoop_Input, UartEventHandler, NULL);
    if (uartEventReg == NULL) {
        return ExitCode_Init_RegisterIo;
    }

    return ExitCode_Success;
}

/// <summary>
///     Closes a file descriptor and prints an error on failure.
/// </summary>
/// <param name="fd">File descriptor to close</param>
/// <param name="fdName">File descriptor name to use in error message</param>
static void CloseFdAndPrintError(int fd, const char *fdName)
{
    if (fd >= 0) {
        int result = close(fd);
        if (result != 0) {
            Log_Debug("ERROR: Could not close fd %s: %s (%d).\n", fdName, strerror(errno), errno);
        }
    }
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    DisposeEventLoopTimer(accelTimer);
    EventLoop_Close(eventLoop);

    Log_Debug("Closing file descriptors.\n");
    CloseFdAndPrintError(i2cFd, "i2c");
    CloseFdAndPrintError(uartFd, "Uart");
}


/// <summary>
///     Helper function to send a fixed message via the given UART.
/// </summary>
/// <param name="uartFd">The open file descriptor of the UART to write to</param>
/// <param name="dataToSend">The data to send over the UART</param>
static void SendUartMessage(int uartFd, const char *dataToSend)
{
    size_t totalBytesSent = 0;
    size_t totalBytesToSend = strlen(dataToSend);
    int sendIterations = 0;
    while (totalBytesSent < totalBytesToSend) {
        sendIterations++;

        // Send as much of the remaining data as possible
        size_t bytesLeftToSend = totalBytesToSend - totalBytesSent;
        const char *remainingMessageToSend = dataToSend + totalBytesSent;
        ssize_t bytesSent = write(uartFd, remainingMessageToSend, bytesLeftToSend);
        if (bytesSent == -1) {
            Log_Debug("ERROR: Could not write to UART: %s (%d).\n", strerror(errno), errno);
            exitCode = ExitCode_SendMessage_Write;
            return;
        }

        totalBytesSent += (size_t)bytesSent;
    }

    Log_Debug("Sent %zu bytes over UART in %d calls.\n", totalBytesSent, sendIterations);
}


/// <summary>
///     Handle UART event: if there is incoming data, print it.
///     This satisfies the EventLoopIoCallback signature.
/// </summary>
static void UartEventHandler(EventLoop *el, int fd, EventLoop_IoEvents events, void *context)
{
    const size_t receiveBufferSize = 256;
    uint8_t receiveBuffer[receiveBufferSize + 1]; // allow extra byte for string termination
    ssize_t bytesRead;

    // Read incoming UART data. It is expected behavior that messages may be received in multiple
    // partial chunks.
    bytesRead = read(uartFd, receiveBuffer, receiveBufferSize);
    if (bytesRead == -1) {
        Log_Debug("ERROR: Could not read UART: %s (%d).\n", strerror(errno), errno);
        exitCode = ExitCode_UartEvent_Read;
        return;
    }

    if (bytesRead > 0) {
        // Null terminate the buffer to make it a valid string, and print it
        receiveBuffer[bytesRead] = 0;
        Log_Debug("UART received %d bytes: '%s'.\n", bytesRead, (char *)receiveBuffer);
    }
}

/// <summary>
///     Main entry point for this application.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("I2C accelerometer application starting.\n");
    exitCode = InitPeripheralsAndHandlers();

    // Use event loop to wait for events and trigger handlers, until an error or SIGTERM happens
    while (exitCode == ExitCode_Success) {
        EventLoop_Run_Result result = EventLoop_Run(eventLoop, -1, true);
        // Continue if interrupted by signal, e.g. due to breakpoint being set.
        if (result == EventLoop_Run_Failed && errno != EINTR) {
            exitCode = ExitCode_Main_EventLoopFail;
        }
    }

    ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return exitCode;
}