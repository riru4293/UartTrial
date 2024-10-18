#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#if 1
#define GP_LED (25U)
#endif

/* UART configuration */
#define UART_PORT_TX (0U)
#define UART_PORT_RX (1U)
#define UART_ID (uart0)
#define UART_IRQ (UART0_IRQ)
#define UART_BAUD_RATE (115200U)
#define UART_DATA_BITS (8U)
#define UART_STOP_BITS (1U)
#define UART_PARITY (UART_PARITY_NONE)

/* Received commands buffer */
#define CMD_BUFF_LEN (2048U)

/* Command part position */
#define CMD_ID_POS (0U)
#define CMD_LEN_POS (2U)
#define CMD_DATA_POS (4U)

/* Command part length */
#define CMD_ID_LEN (2U)
#define CMD_LEN_LEN (2U)
#define CMD_HEAD_LEN (CMD_ID_LEN + CMD_LEN_LEN)
#define CMD_LEN_MAX (1024U)
#define CMD_DATA_LEN_MAX (CMD_LEN_MAX - CMD_ID_LEN - CMD_LEN_LEN)

typedef struct
{
    uint16_t id;
    uint16_t len;
    uint8_t data[CMD_DATA_LEN_MAX];
} stCmd;

#if 1
static void blinkTask(void *nouse);
#endif

static void onRecvUart(void);
static void recvUartTask(void *nouse);
static void procCmdTask(void *nouse);
static void queueCmd(const size_t n, const uint8_t cmd[n]);
static bool dequeueCmd(stCmd *cmd);
static bool isEmptyCmdBuff(void);

#if 1
static const uint16_t C_TASK_INTERVAL = 500U;
static TaskHandle_t gBlinkTask = NULL;
static TaskHandle_t gUartTask = NULL;
static bool gLedOn = true;
#endif

static TaskHandle_t gRecvUartTask = NULL;
static TaskHandle_t gProcCmdTask = NULL;
static uint8_t gCmdBuff[CMD_BUFF_LEN] = {0U};
static size_t gCmdReadIdx = 0U;
static size_t gCmdWriteIdx = 0U;
static bool gIsCmdBuffFull = false;
static uint8_t gRecvBuff[CMD_LEN_MAX] = {0U};

int main(void)
{
    (void)uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_PORT_TX, UART_FUNCSEL_NUM(UART_ID, UART_PORT_TX));
    gpio_set_function(UART_PORT_RX, UART_FUNCSEL_NUM(UART_ID, UART_PORT_RX));
    uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    uart_set_hw_flow(UART_ID, false, false);

    /* Enable UART IRQ handling */
    irq_set_exclusive_handler(UART_IRQ, onRecvUart);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    (void)xTaskCreate(blinkTask, "Blink", 1024, NULL, 3, &gBlinkTask);
    (void)xTaskCreate(recvUartTask, "UartRx", 1024, NULL, 2, &gRecvUartTask);
    (void)xTaskCreate(procCmdTask, "CmdProc", 1024, NULL, 1, &gProcCmdTask);
    vTaskStartScheduler();

    while (1)
    {
        /* Unreachable */
    }
}

#if 1
static void blinkTask(void *nouse)
{
    gpio_init(GP_LED);
    gpio_set_dir(GP_LED, GPIO_OUT);

    while (1)
    {
        gpio_put(GP_LED, gLedOn);
        vTaskSuspendAll();
        vTaskDelay(C_TASK_INTERVAL);
        (void)xTaskResumeAll();
        vTaskDelay(C_TASK_INTERVAL);
        gLedOn = !gLedOn;
    }
}
#endif

/**
 * @brief Notify UART received.
 */
static void onRecvUart(void)
{
    BaseType_t hptw = pdFALSE;

    /* Notify UART received */
    vTaskNotifyGiveFromISR(gRecvUartTask, &hptw);

    /* Exit to context switch if necessary */
    portYIELD_FROM_ISR(hptw);
}

/**
 * @brief UART receive interrupt handler.
 * @details It receives data in ASCII format, converts it to binary and stores
 * it in an internal buffer. When the data stored in the internal buffer
 * is completed as one command, it moves the received command to a command
 * buffer and prepares to receive the next command.
 * @param nouse no used
 */
static void recvUartTask(void *nouse)
{
    /* UART control characters */
    typedef enum
    {
        E_START_MARKER = '<',
        E_END_MARKER = '>'
    } enUartCtrlChar;

    /* Defining a Hexadecimal ASCII characters buffer */
    /* Note: ASCII characters 0 to F are received, and every two characters */
    /*       received are converted to 1 byte of binary.                    */
    typedef enum
    {
        E_ASCII_MSB_POS,
        E_ASCII_LSB_POS,
        E_ASCII_QTY
    } enAsciiHex;

    static size_t buffIdx = 0U;
    static bool nowLoading = false;
    static bool isReadMsb = true;

    while (true)
    {
        /* Wait receive UART */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (uart_is_readable(UART_ID))
        {
            /* Receive 1 character. */
            uint8_t ch = uart_getc(UART_ID);

            if (E_START_MARKER == ch)
            {
                /* Begins buffering incoming characters. */
                nowLoading = true;
                isReadMsb = true;
                buffIdx = 0U;
            }
            else if (nowLoading)
            {
                if ((E_END_MARKER == ch) && (CMD_HEAD_LEN <= buffIdx) && isReadMsb)
                {
                    /* Moves the received contents to the command buffer */
                    /* and ends buffering of received characters.        */
                    queueCmd(buffIdx, gRecvBuff);
                    nowLoading = false;
                }
                else if (isxdigit(ch) && (CMD_LEN_MAX > buffIdx))
                {
                    static uint8_t ascii[E_ASCII_QTY];

                    if (isReadMsb)
                    {
                        /* Stores the upper digits. */
                        ascii[E_ASCII_MSB_POS] = ch;
                    }
                    else
                    {
                        /* Converts two characters in ASCII format to binary format. */
                        ascii[E_ASCII_LSB_POS] = ch;
                        sscanf(ascii, "%hhX", &gRecvBuff[buffIdx++]);
                    }

                    /* Swaps the upper and lower digits of the number to be read. */
                    isReadMsb = !isReadMsb;
                }
                else
                {
                    /* Terminates buffering of incoming characters if unexpected. */
                    nowLoading = false;
                }
            }
        }
    }
}

/**
 * @brief Process command.
 * 
 * @param nouse no used
 */
static void procCmdTask(void *nouse)
{
    stCmd cmd = {.id = 0U, .len = 0U, .data = {0U}};

    while (true)
    {
        /* Wait a command queuing. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dequeueCmd(&cmd))
        {
            switch (cmd.id)
            {
            case 0x1234:
                break;

            default:
                break;
            }
        }
    }
}

/**
 * @brief Store a command to command buffer.
 *
 * @param [in] n command length
 * @param [in] cmd command
 */
static void queueCmd(const size_t n, const uint8_t cmd[n])
{
    if ((0U < n) && (NULL != cmd) && !gIsCmdBuffFull)
    {
        /* Calculate free space size */
        const size_t free = (gCmdReadIdx <= gCmdWriteIdx)
                                ? CMD_BUFF_LEN - (gCmdWriteIdx - gCmdReadIdx)
                                : gCmdReadIdx - gCmdWriteIdx;

        /* Note: gCmdWriteIdx + n is checked to ensure it does not overflow, but it never does. */
        if ((n <= free) && ((SIZE_MAX - n) >= gCmdWriteIdx))
        {
            /* Copy to command buffer */
            if (CMD_BUFF_LEN >= (gCmdWriteIdx + n))
            {
                /* If it fits to the end position of the command buffer */
                memcpy(&gCmdBuff[gCmdWriteIdx], cmd, n);
            }
            else
            {
                /* If the end of the command buffer is exceeded */
                size_t s = CMD_BUFF_LEN - gCmdWriteIdx;
                memcpy(&gCmdBuff[gCmdWriteIdx], cmd, s); /* To end position     */
                memcpy(gCmdBuff, &cmd[s], n - s);        /* From start position */
            }

            /* Forward the writing start position of the command buffer */
            gCmdWriteIdx = (gCmdWriteIdx + n) % CMD_BUFF_LEN;

            /* When the write start position catches up         */
            /* with the read start position, the buffer is full */
            gIsCmdBuffFull = (gCmdReadIdx == gCmdWriteIdx);

            /* Notify command received */
            (void)xTaskNotifyGive(gRecvUartTask);
        }
    }
}

/**
 * @brief Get a command from command buffer.
 * @details The command format is as follows.
 * position | length | endian | description
 * -------: | -----: | :----- | :----------
 * 0        | 2      | little | Command ID
 * 2        | 2      | little | Data length
 * 4        | 1020   | big    | Data. The remaining area is indefinite value.
 *
 * @param [out] cmd the stCmd
 * @return true buffered command found
 * @return false buffered command not found
 */
static bool dequeueCmd(stCmd *cmd)
{
    if ((NULL != cmd) && (NULL != cmd->data) && !isEmptyCmdBuff())
    {
        /* Note: gCmdReadIdx + CMD_HEAD_LEN is checked to ensure it does not overflow, but it never does. */
        if ((SIZE_MAX - CMD_HEAD_LEN) >= gCmdReadIdx)
        {
            uint8_t hdr[CMD_HEAD_LEN] = {0U};
            uint16_t id = 0U;
            uint16_t len = 0U;
            const size_t dataPos = gCmdReadIdx + CMD_HEAD_LEN;

            /* == Get command id and command data length. == */
            for (uint8_t i = 0U; i < CMD_HEAD_LEN; i++)
            {
                hdr[i] = gCmdBuff[(gCmdReadIdx + i) % CMD_BUFF_LEN];
            }

            memcpy(&id, &hdr[CMD_ID_POS], CMD_ID_LEN);
            memcpy(&len, &hdr[CMD_LEN_POS], CMD_LEN_LEN);
            /* ============================================= */

            /* Note: gCmdReadIdx + CMD_HEAD_LEN + dataPos is checked to ensure */
            /*       it does not overflow, but it never does.                  */
            if ((CMD_DATA_LEN_MAX >= len) && ((SIZE_MAX - len) >= dataPos))
            {
                /* == Get command data. == */
                if (CMD_BUFF_LEN >= (dataPos + len))
                {
                    /* If it fits to the end position of the command buffer */
                    memcpy(cmd->data, &gCmdBuff[dataPos], len);
                }
                else
                {
                    /* If the end of the command buffer is exceeded */
                    size_t s = CMD_BUFF_LEN - dataPos;
                    memcpy(cmd->data, &gCmdBuff[dataPos], s); /* To end position     */
                    memcpy(&cmd->data[s], gCmdBuff, len - s); /* From start position */
                }
                /* ======================= */

                /* Forward the reading start position of the command buffer. */
                gCmdReadIdx = (dataPos + len) % CMD_BUFF_LEN;

                /* Command buffer has free space. */
                gIsCmdBuffFull = false;

                /* Successful. */
                return true;
            }
        }
    }

    /* Failure. */
    return false;
}

/**
 * @brief Return true if command buffer has free space
 *
 * @return true command buffer has free space
 * @return false command buffer is full
 */
static bool isEmptyCmdBuff(void)
{
    return !gIsCmdBuffFull && (gCmdReadIdx == gCmdWriteIdx);
}
