#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <UartRx.h>

static void task(void *nouse);

static TaskHandle_t gTask;

/**
 * @brief Create a task that receive by UART.
 */
void Init(void)
{
    (void)xTaskCreate(task, "UartRx", 1024, NULL, 1, &gTask);
}

/**
 * @brief
 *
 * @param nouse
 */
static void task(void *nouse)
{
}