#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#define GP_LED                      ( 25 )

static void task(void *nouse);

static const uint16_t C_TASK_INTERVAL = 500U;
static TaskHandle_t gTask = NULL;
static bool gLedOn = true;

int main(void)
{
    BaseType_t result = xTaskCreate(task, "Blink", 1024, NULL, 2, &gTask);
    vTaskStartScheduler();

    while (1)
    {
        /* Unreachable */
    }
}

static void task(void *nouse)
{
    gpio_init( GP_LED );
    gpio_set_dir( GP_LED, GPIO_OUT );

    while (1)
    {
        gpio_put( GP_LED, gLedOn );
        vTaskSuspendAll();
        vTaskDelay(C_TASK_INTERVAL);
        (void)xTaskResumeAll();
        vTaskDelay(C_TASK_INTERVAL);
        gLedOn = !gLedOn;
    }
}