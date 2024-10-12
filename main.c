#include <string.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#define GP_LED (25U)
#define UART_TX_PIN (0U)
#define UART_RX_PIN (1U)
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

typedef struct
{
    uint16_t cmd;
    uint16_t len;
    uint8_t data[100U];
} stUartCmd;

static void blinkTask(void *nouse);
static void uartTask(void *nouse);
static void onUartRx(void);

static const uint16_t C_TASK_INTERVAL = 500U;
static TaskHandle_t gBlinkTask = NULL;
static TaskHandle_t gUartTask = NULL;
static bool gLedOn = true;
static volatile QueueHandle_t gQueue = NULL;

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
    const UBaseType_t C_QUE_ITEM_QTY = 10U;
    const UBaseType_t C_QUE_ITEM_SIZE = sizeof(stUartCmd);

    gQueue = xQueueCreate(C_QUE_ITEM_QTY, C_QUE_ITEM_SIZE);
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

    uart_puts(UART_ID, "\nHello, gQueue interrupts\n");

    while (1)
    {
        tight_loop_contents();
    }
}

static void onUartRx(void)
{
    static uint8_t step = 0U;
    static uint8_t buff[100U] = {0U};
    static uint8_t idx = 0U;
    stUartCmd uartCmd;
    uart_hw_t *uart = uart_get_hw(UART_ID);

    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);

        switch (ch)
        {
        case '<':
            step = 1U;
            idx = 0U;
            break;

        case '>':
            memcpy(&uartCmd.cmd, &buff[0U], 2U);
            memcpy(&uartCmd.len, &buff[2U], 2U);
            memcpy(uartCmd.data, &buff[4U], 96U);

            xQueueSendToBack(gQueue, &uartCmd, 0U);
            step = 0U;
            idx = 0U;
            break;

        default:
            if (0U != step)
            {
                buff[idx] = ch;
                idx++;
            }
            break;
        }
    }
}