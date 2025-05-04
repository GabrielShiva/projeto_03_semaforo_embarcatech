#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "lib/ws2818b.h"
#include "ws2818b.pio.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>

// Definição de macros para o protocolo I2C (SSD1306)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

#define BTN_B_PIN 6
#define BTN_A_PIN 5
#define BUZZER_PIN 21

#define led1 11
#define led2 12

// define variável global para trocar modo de operação (NORMAL e NOTURNO)
volatile bool is_normal_mode = true;

// define variáveis para debounce do botão
volatile uint32_t last_time_btn_press = 0;
const uint32_t debounce_delay_ms = 260;

void btn_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
}

void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

void buzzer_init() {
  // Configura o pino do buzzer para PWM
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
  uint channel_num = pwm_gpio_to_channel(BUZZER_PIN);

  // Configuração inicial do PWM
  pwm_config config = pwm_get_default_config();
  pwm_init(slice_num, &config, true);
  pwm_set_enabled(slice_num, false); // desliga PWM do pino ligado ao buzzer
}

void define_buzzer_state(float buzzer_freq) {
  // if(led_rgb_state && volume_scale > 0) {
  //   buzzer_freq = 200.0f + (volume_scale - 1) * 200.0f;

  //   // Cálculos para configuração do PWM
  //   uint32_t clock = 125000000; // Clock base de 125MHz
  //   uint32_t divider = 125000000 / (uint32_t)(buzzer_freq * 1000);
  //   uint32_t wrap = 125000000 / (divider * (uint32_t)buzzer_freq) - 1;

  //   // Aplica as configurações
  //   pwm_set_clkdiv_int_frac(slice_num, divider, 0);
  //   pwm_set_wrap(slice_num, wrap);
  //   pwm_set_chan_level(slice_num, channel_num, wrap / 2); // Define o Duty cycle de 50%
  //   pwm_set_enabled(slice_num, true);
  // } else {
  //   // Desliga o PWM
  //   pwm_set_enabled(slice_num, false);
  //   gpio_put(BUZZER_PIN, 0); // Garante o silêncio
  // }
}

void vBlinkLed1Task() {
    gpio_init(led1);
    gpio_set_dir(led1, GPIO_OUT);
    while (true)
    {
        gpio_put(led1, true);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_put(led1, false);
        vTaskDelay(pdMS_TO_TICKS(1223));
    }
}

void vBlinkLed2Task() {
    gpio_init(led2);
    gpio_set_dir(led2, GPIO_OUT);
    while (true)
    {
        gpio_put(led2, true);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(led2, false);
        vTaskDelay(pdMS_TO_TICKS(2224));
    }
}

void vDisplayTask() {
    // Inicialização do protocolo I2C com 400Khz
    i2c_setup(400);

    // Inicializa a estrutura do display
    ssd1306_t ssd;
    ssd1306_setup(&ssd);

    char str_y[5]; // Buffer para armazenar a string
    int contador = 0;
    bool cor = true;
    while (true)
    {
        sprintf(str_y, "%d", contador); // Converte em string
        contador++;                     // Incrementa o contador
        ssd1306_fill(&ssd, !cor);                          // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);      // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);           // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);           // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6); // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);  // Desenha uma string
        ssd1306_draw_string(&ssd, "  FreeRTOS", 10, 28); // Desenha uma string
        ssd1306_draw_string(&ssd, "Contador  LEDs", 10, 41);    // Desenha uma string
        ssd1306_draw_string(&ssd, str_y, 40, 52);          // Desenha uma string
        ssd1306_send_data(&ssd);                           // Atualiza o display
        sleep_ms(735);
    }
}

void vLedMatrixTask() {
  // Inicializa matriz de LEDs
  npInit(LED_PIN);
  npClear();
  npSetBrightness(255);
  npWrite();

  while (true) {
  }
}

void vBuzzerTask() {
  float buzzer_freq = 0.0f;

  buzzer_init();

  while (true) {
    define_buzzer_state(buzzer_freq);
  }
}

void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_ms_since_boot(get_absolute_time()); // retorna o tempo total em ms desde o boot do rp2040

  // verifica se a diff entre o tempo atual e a ultima vez que o botão foi pressionado é maior que o tempo de debounce
  if (current_time - last_time_btn_press > debounce_delay_ms) {
    last_time_btn_press = current_time;

    if (gpio == BTN_A_PIN) {
      printf("botao A!\n");
    } else if (gpio == BTN_B_PIN) {
      reset_usb_boot(0, 0);
    }
  }
}

int main() {
  stdio_init_all();

  // Inicilização dos botões
  btn_setup(BTN_A_PIN);
  btn_setup(BTN_B_PIN);

  // Inicilização das interrupções para os botões
  gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true);

  // Criação das tarefas
  xTaskCreate(vBlinkLed1Task, "Blink Task Led1", configMINIMAL_STACK_SIZE,
        NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vBlinkLed2Task, "Blink Task Led2", configMINIMAL_STACK_SIZE,
      NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vDisplayTask, "Task Display", configMINIMAL_STACK_SIZE,
      NULL, tskIDLE_PRIORITY, NULL);

  // Chamda do Scheduller de tarefas
  vTaskStartScheduler();
  panic_unsupported();
}
