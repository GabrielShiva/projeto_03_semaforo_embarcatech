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

#define LED_RED 13
#define LED_GREEN 11
#define LED_BLUE 12

#define STEP_MS   100   // check every 100 ms

// Define variáveis para debounce dos botões A e B
volatile uint32_t last_time_btn_press = 0;
const uint32_t debounce_delay_ms = 260;

// define variável global para trocar modo de operação (NORMAL e NOTURNO)
volatile bool is_night_mode = false;
volatile uint counter = 0;
volatile uint traffic_light_step = 0; // 0=>verde ; 1=>amarelo ; 2=>vermelho

// pwm
uint32_t clock   = 125000000;
uint32_t divider = 0;
uint32_t wrap    = 0;
uint slice_num   = 0;
uint channel_num = 0;
uint ledsMatrixCounter = 0;

void setMatrixColor() {
  for (uint i = 0; i < LED_COUNT; ++i) {
    npSetLED(i, 255, 255, 255);
  }
}

void btn_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
}

void led_rgb_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
}

void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

void pwm_set_frequency(float frequency) {
  if (frequency <= 0.0f) {
    pwm_set_enabled(slice_num, false);
    return;
  }

  divider = clock / (uint32_t)(frequency * 1000);
  wrap    = clock / (divider * (uint32_t)frequency) - 1;

  // Aplica as configurações
  pwm_set_clkdiv_int_frac(slice_num, divider, 0);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel_num, wrap / 2); // Define o Duty cycle de 50%
}

void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

void buzzer_set_delay(uint delay_ms, uint slice, bool is_for_night_mode) {
  if (is_for_night_mode) {
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (!is_night_mode) {
        pwm_set_enabled(slice, false);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
  } else {
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (is_night_mode) {
        pwm_set_enabled(slice, false);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
  }
}

void leds_set_delay(uint delay_ms, bool is_for_night_mode) {
  if (is_for_night_mode) {
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (!is_night_mode) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
  } else {
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (is_night_mode) {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
  }
}

void vDisplayTask() {
  // Inicialização do protocolo I2C com 400Khz
  i2c_setup(400);

  // Inicializa a estrutura do display
  ssd1306_t ssd;
  ssd1306_setup(&ssd);

  bool color = true;

  while (true) {
    ssd1306_fill(&ssd, !color);
    ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color);
    ssd1306_line(&ssd, 3, 15, 123, 15, color);
    ssd1306_line(&ssd, 3, 27, 123, 27, color);
    ssd1306_draw_string(&ssd, "SEMAFORO", 32, 6);

    if (is_night_mode) {
      ssd1306_draw_string(&ssd, "MODO: NOTURNO", 6, 18);

      if (counter == 0) {
        ssd1306_draw_string(&ssd, "INICIALIZANDO", 6, 41);
        ssd1306_draw_string(&ssd, "Modo Noturno", 6, 52);
      }

      if (counter > 0) {
        ssd1306_draw_string(&ssd, "Atencao", 6, 41);
      }
    }

    if (!is_night_mode) {
      ssd1306_draw_string(&ssd, "MODO: NORMAL", 6, 18);

      if (counter == 0) {
        ssd1306_draw_string(&ssd, "INICIALIZANDO", 6, 41);
        ssd1306_draw_string(&ssd, "Modo Normal", 6, 52);
      }

      if (counter > 0) {
        if (traffic_light_step == 0) {
          ssd1306_draw_string(&ssd, "Passagem", 6, 41);
          ssd1306_draw_string(&ssd, "Permitida", 6, 52);
        }

        if (traffic_light_step == 1) {
          ssd1306_draw_string(&ssd, "Atencao", 6, 41);
        }

        if (traffic_light_step == 2) {
          ssd1306_draw_string(&ssd, "Passagem", 6, 41);
          ssd1306_draw_string(&ssd, "Proibida", 6, 52);
        }
      }
    }

    ssd1306_send_data(&ssd);
  }
}

void vBuzzerTask() {
  // Configura o pino do buzzer para PWM e obtém as infos do pino
  gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
  slice_num   = pwm_gpio_to_slice_num(BUZZER_PIN);
  channel_num = pwm_gpio_to_channel(BUZZER_PIN);

  // Configuração inicial do PWM
  pwm_config config = pwm_get_default_config();
  pwm_init(slice_num, &config, true);

  // Desliga PWM do pino ligado ao buzzer
  pwm_set_enabled(slice_num, false);

  while (true) {
    if (is_night_mode) {
      // Cálculos para configuração do PWM
      pwm_set_frequency(100.0f);

      if (counter > 0) {
        // Ligado
        pwm_set_enabled(slice_num, true);
        buzzer_set_delay(2000, slice_num, true);

        // Desligado
        pwm_set_enabled(slice_num, false);
        buzzer_set_delay(2000, slice_num, true);
      }
    } else {
      if (counter > 0) {
        // Verde (8 segundos)
        if (traffic_light_step == 0) {
          pwm_set_frequency(200.0f);

          // Ligado
          pwm_set_enabled(slice_num, true);
          buzzer_set_delay(1000, slice_num, false);

          // Desligado
          pwm_set_enabled(slice_num, false);
          buzzer_set_delay(7000, slice_num, false);
        }

        // Amarelo (4 segundos)
        if (traffic_light_step == 1) {
          pwm_set_frequency(100.0f); // Define a frequência

          // Ligado
          pwm_set_enabled(slice_num, true);
          buzzer_set_delay(500, slice_num, false);

          // Desligado
          pwm_set_enabled(slice_num, false);
          buzzer_set_delay(500, slice_num, false);
        }

        // Vermelho (8 segundos)
        if (traffic_light_step == 2) {
          pwm_set_frequency(500.0f); // Define a frequência

          // Ligado
          pwm_set_enabled(slice_num, true);
          buzzer_set_delay(500, slice_num, false);

          // Desligado
          pwm_set_enabled(slice_num, false);
          buzzer_set_delay(1500, slice_num, false);
        }
      }
    }
  }
}

void vLedMatrixTask() {
  npInit(LED_PIN);
  npClear();
  npWrite();

  const TickType_t step = pdMS_TO_TICKS(STEP_MS);
  const uint stepsPer = 2000 / STEP_MS;

  while (true) {
    if (!is_night_mode) {
      if (counter > 0 && traffic_light_step == 0 && ledsMatrixCounter < 4) {
        // Liga
        setMatrixColor();
        npWrite();
        leds_set_delay(200, false);

        // Desliga
        npClear();
        npWrite();
        leds_set_delay(200, false);

        ledsMatrixCounter++;
      }
    } else {
      npClear();
      npWrite();
    }
  }
}

void vLEDsRGBTask() {
  led_rgb_setup(LED_RED);
  led_rgb_setup(LED_GREEN);
  led_rgb_setup(LED_BLUE);

  while (true) {
    if (is_night_mode) {
      // Entrando no modo noturno
      if (counter == 0) {
        gpio_put(LED_RED, false);
        gpio_put(LED_GREEN, false);
        gpio_put(LED_BLUE, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        counter++;
      }

      printf("contador: %d\n", counter);

      // Night ON phase (2 s total)
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, true);
      leds_set_delay(2000, true);

      // Night OFF phase (2 s total)
      gpio_put(LED_RED,   false);
      gpio_put(LED_GREEN, false);
      leds_set_delay(2000, true);
    } else {
      // Entrando no modo normal
      if (counter == 0) {
        gpio_put(LED_RED, false);
        gpio_put(LED_GREEN, false);
        gpio_put(LED_BLUE, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        counter++;
      }

      printf("contador: %d\n", counter);

      // Verde por 8s
      traffic_light_step = 0;
      gpio_put(LED_RED,   false);
      gpio_put(LED_GREEN, true);
      leds_set_delay(8000, false);

      // Amarelo por 4s
      traffic_light_step = 1;
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, true);
      leds_set_delay(4000, false);

      // Vermelho por 8s
      traffic_light_step = 2;
      ledsMatrixCounter = 0;
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, false);
      leds_set_delay(8000, false);
    }
  }
}

void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_ms_since_boot(get_absolute_time()); // retorna o tempo total em ms desde o boot do rp2040

  // verifica se a diff entre o tempo atual e a ultima vez que o botão foi pressionado é maior que o tempo de debounce
  if (current_time - last_time_btn_press > debounce_delay_ms) {
    last_time_btn_press = current_time;

    if (gpio == BTN_A_PIN) {
      counter = 0;
      is_night_mode = !is_night_mode;
      traffic_light_step = 0;
      ledsMatrixCounter = 0;
      pwm_set_enabled(slice_num, false);

      npClear();
      npWrite();

      printf("contador: %d\n", counter);

      if (is_night_mode) {
        printf("Modo Atual: Noturno.\n");
      } else {
        printf("Modo Atual: Normal.\n");
      }
    } else if (gpio == BTN_B_PIN) {
        reset_usb_boot(0, 0);
    }
  }
}

int main() {
  stdio_init_all();

  // Inicilização dos botões
  btn_setup(BTN_B_PIN);
  btn_setup(BTN_A_PIN);

  // Inicilização das interrupções para os botões
  gpio_set_irq_enabled_with_callback(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
  gpio_set_irq_enabled(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true);

  // Criação das tarefas
  xTaskCreate(vDisplayTask, "Task: Display", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vLedMatrixTask, "Task: LEDs Matriz", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vBuzzerTask, "Task: Buzzer", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(vLEDsRGBTask, "Task: LEDs RGB", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

  // Chamda do Scheduller de tarefas
  vTaskStartScheduler();
  panic_unsupported();
}
