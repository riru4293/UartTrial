#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <task.h>

#define GP_LED (25U)
#define UART_TX_PIN (0U)
#define UART_RX_PIN (1U)
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

static void blinkTask(void *nouse);
static void uartTask(void *nouse);
static void onUartRx(void);

static const uint16_t C_TASK_INTERVAL = 500U;
static TaskHandle_t gBlinkTask = NULL;
static TaskHandle_t gUartTask = NULL;
static bool gLedOn = true;

int main(void)
{
    (void)xTaskCreate(blinkTask, "Blink", 1024, NULL, 2, &gBlinkTask);
    (void)xTaskCreate(uartTask, "Uart", 1024, NULL, 1, &gUartTask);
    vTaskStartScheduler();

    while (1)
    {
        /* Unreachable */
    }
}

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

static void uartTask(void *nouse)
{
    (void)uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    uart_set_hw_flow(UART_ID, false, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, onUartRx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    uart_puts(UART_ID, "\nHello, uart interrupts\n");

    while (1)
    {
        tight_loop_contents();
    }
}

#define UART_CMD_LEN_MAX (1024U)
#define UART_CMD_START_MARK ('<')
#define UART_CMD_END_MARK ('>')
#define UART_CMD_CODE_POS (0U)
#define UART_CMD_LEN_POS (2U)
#define UART_CMD_CODE_LEN (2U)
#define UART_CMD_LEN_LEN (2U)
#define UART_CMD_HEAD_LEN (UART_CMD_CODE_LEN + UART_CMD_LEN_LEN)

static void onUartRx(void)
{
    static uint16_t idx = 0U;
    static uint8_t buff[UART_CMD_LEN_MAX];
    static bool nowReading = false;
    static bool isMsb = true;

    uart_hw_t *uart = uart_get_hw(UART_ID);

    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);

        if (UART_CMD_START_MARK == ch)
        {
            nowReading = true;
            isMsb = true;
            idx = 0U;
        }
        else if (nowReading)
        {
            if ((UART_CMD_END_MARK == ch) && isMsb && (UART_CMD_HEAD_LEN <= idx))
            {
                // copy to ring-buff
                uint16_t cmd = 0U;
                uint16_t cmdLen = 0U;
                memcpy(&cmd, &buff[UART_CMD_CODE_POS], sizeof(cmd));
                memcpy(&cmdLen, &buff[UART_CMD_LEN_POS], sizeof(cmdLen));

                if (cmdLen == (uint16_t)(idx - UART_CMD_HEAD_LEN))
                {
                    printf("cmd: %04X, len: %04X, data: ", cmd, cmdLen);
                    for (uint8_t i = 4; i < idx; i++)
                        printf("%02X ", buff[i]);
                    printf("\n");
                }
            }
            else if (isxdigit(ch) && (UART_CMD_LEN_MAX > idx))
            {
                static uint8_t ascii[2];

                if (true == isMsb)
                {
                    ascii[0] = ch;
                }
                else
                {
                    ascii[1] = ch;
                    sscanf(ascii, "%hhX", &buff[idx++]);
                }

                isMsb = !isMsb;
            }
            else
            {
                nowReading = false;
            }
        }
    }
}