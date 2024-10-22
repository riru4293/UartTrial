#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/irq.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <class/cdc/cdc_device.h>

int main() {
    stdio_init_all();

    unsigned short cmd_buff_index_ = 0;
    char cmd_buff[1024] = {0};
    char tmp_input;
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        // RxBuffにデータが存在する
        if(tud_cdc_available() > 0){
            // 1文字受信
            tmp_input = tud_cdc_read_char();

            tud_cdc_write_char(tmp_input);
            tud_cdc_write_flush();

            // 改行文字か？
            if(tmp_input == '\r') {
                // null文字付与
                cmd_buff[cmd_buff_index_] = '\0';
                // コマンド判断
                char* command = &cmd_buff[0];
                if(strcmp(command, "led") == 0) {
                    printf("LED Blink\n");
                    gpio_put(LED_PIN, 1);
                    sleep_ms(250);
                    gpio_put(LED_PIN, 0);
                    sleep_ms(250);
                } else {
                    printf("%s", command);
                    printf("no such command\n");
                }
                // index clear
                cmd_buff_index_ = 0;
            } else {
                cmd_buff[cmd_buff_index_] = tmp_input;
                cmd_buff_index_++;
            }
        }
        // wait
        sleep_ms(1);
    }
}

void tud_cdc_rx_cb(uint8_t itf) {
    printf("hello");
}
