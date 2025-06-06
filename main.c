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

#define STEP_MS 100

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

// Define todos o buffer para os LEDs da matriz como brancos
void setMatrixColor() {
  for (uint i = 0; i < LED_COUNT; ++i) {
    npSetLED(i, 255, 255, 255);
  }
}

// Realiza a inicialização dos botões
void btn_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
}

// Realiza a inicialização dos LEDs RGB
void led_rgb_setup(uint gpio) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_OUT);
}

// Realiza a inicialização do protocolo I2C para comunicação com o display OLED
void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

// Cálculo dos paramêtros do PWM para buzzer emitir frequência especificada
void pwm_set_frequency(float frequency) {
  // Se frequência for menor que zero não executa nada
  if (frequency <= 0.0f) {
    pwm_set_enabled(slice_num, false);
    return;
  }

  // Calcula os valores para o divisor e para o wrap
  divider = clock / (uint32_t)(frequency * 1000);
  wrap    = clock / (divider * (uint32_t)frequency) - 1;

  // Aplica as configurações calculados
  pwm_set_clkdiv_int_frac(slice_num, divider, 0);
  pwm_set_wrap(slice_num, wrap);
  pwm_set_chan_level(slice_num, channel_num, wrap / 2); // Define o Duty cycle de 50%
}

// Realiza a inicialização do display OLED
void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

// Aplica o delay para o buzzer em passos e com verificação do estado da flag is_night_mode (evita que a tarefa fique presa no delay).
void buzzer_set_delay(uint delay_ms, uint slice, bool is_for_night_mode) {
  if (is_for_night_mode) { // verifica se o modo é noturno. Se sim, executa o bloco abaixo.
    // Realiza a verificação se a flag de estado do semaforo mudou. Sem sim. Interrompe o loop. Se não, continua a aplicar o delay para o buzzer.
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (!is_night_mode) { // se a flag tiver mudado para modo normal, desliga o buzzer e interrompe o for.
        pwm_set_enabled(slice, false);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS)); // se flag não mudou aplica o delay de 100ms.
    }
  } else {
    // Realiza a verificação se a flag de estado do semaforo mudou. Sem sim. Interrompe o loop. Se não, continua a aplicar o delay para o buzzer.
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (is_night_mode) { // se a flag tiver mudado para modo noturno, desliga o buzzer e interrompe o for.
        pwm_set_enabled(slice, false);
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS)); // se flag não mudou aplica o delay de 100ms.
    }
  }
}

// Aplica o delay para os LEDs RGB (semaforo) em passos e com verificação do estado da flag is_night_mode (evita que a tarefa fique presa no delay).
void leds_set_delay(uint delay_ms, bool is_for_night_mode) {
  if (is_for_night_mode) { // verifica se o modo é noturno. Se sim, executa o bloco abaixo.
    // Realiza a verificação se a flag de estado do semaforo mudou. Sem sim. Interrompe o loop. Se não, continua a aplicar o delay para o LED.
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (!is_night_mode) {  // se a flag tiver mudado para modo normal, interrompe o for.
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS)); // se flag não mudou aplica o delay de 100ms.
    }
  } else {
    // Realiza a verificação se a flag de estado do semaforo mudou. Sem sim. Interrompe o loop. Se não, continua a aplicar o delay para o LED.
    for (int i = 0; i < delay_ms; i += STEP_MS) {
      if (is_night_mode) { // se a flag tiver mudado para modo noturno, interrompe o for.
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(STEP_MS));  // se flag não mudou aplica o delay de 100ms.
    }
  }
}

// Implementa a tarefa do display OLED
void vDisplayTask() {
  // Inicialização do protocolo I2C com 400Khz
  i2c_setup(400);

  // Inicializa a estrutura do display
  ssd1306_t ssd;
  ssd1306_setup(&ssd);

  bool color = true;

  while (true) {
    // Realiza a limpeza do display e define o layout do display OLED
    ssd1306_fill(&ssd, !color);
    ssd1306_rect(&ssd, 3, 3, 122, 60, color, !color);
    ssd1306_line(&ssd, 3, 15, 123, 15, color);
    ssd1306_line(&ssd, 3, 27, 123, 27, color);
    ssd1306_draw_string(&ssd, "SEMAFORO", 32, 6);

    // Se o modo for noturno, executa o bloco de código abaixo
    if (is_night_mode) {
      ssd1306_draw_string(&ssd, "MODO: NOTURNO", 6, 18);

      // se o usuário pressiona o botão, counter é igual à zero e a mensagem de inicialização do modo é exibida
      if (counter == 0) {
        ssd1306_draw_string(&ssd, "INICIALIZANDO", 6, 41);
        ssd1306_draw_string(&ssd, "Modo Noturno", 6, 52);
      }

      // Após o delay de 3s aplicado, a variável counter se torna maior que zero e o sinal começa a operar no modo estabelecido.
      // Com isso, é exibido a mensagem do sinal amarelo.
      if (counter > 0) {
        ssd1306_draw_string(&ssd, "Atencao", 6, 41);
      }
    }

    // Se o modo for normal, executa o bloco de código abaixo
    if (!is_night_mode) {
      ssd1306_draw_string(&ssd, "MODO: NORMAL", 6, 18);

      // se o usuário pressiona o botão, counter é igual à zero e a mensagem de inicialização do modo é exibida
      if (counter == 0) {
        ssd1306_draw_string(&ssd, "INICIALIZANDO", 6, 41);
        ssd1306_draw_string(&ssd, "Modo Normal", 6, 52);
      }

      // Após o delay de 3s aplicado, a variável counter se torna maior que zero e o sinal começa a operar no modo estabelecido.
      if (counter > 0) {

        // Quando o sinal é verde, traffic_light_step é igual a zero e a mensagem abaixo é exibida
        if (traffic_light_step == 0) {
          ssd1306_draw_string(&ssd, "Passagem", 6, 41);
          ssd1306_draw_string(&ssd, "Permitida", 6, 52);
        }

        // Quando o sinal é verde, traffic_light_step é igual a um e a mensagem abaixo é exibida
        if (traffic_light_step == 1) {
          ssd1306_draw_string(&ssd, "Atencao", 6, 41);
        }

        // Quando o sinal é verde, traffic_light_step é igual a dois e a mensagem abaixo é exibida
        if (traffic_light_step == 2) {
          ssd1306_draw_string(&ssd, "Passagem", 6, 41);
          ssd1306_draw_string(&ssd, "Proibida", 6, 52);
        }
      }
    }

    // Envia os dados armazenados no buffer para o display OLED
    ssd1306_send_data(&ssd);
  }
}

// Implementa a tarefa do buzzer
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
    if (is_night_mode) { // verifica se o modo é noturno
      // Cálculos para configuração do PWM
      pwm_set_frequency(100.0f);

      // verifica se já se passaram os 3 segundos
      if (counter > 0) {
        // Ligado
        pwm_set_enabled(slice_num, true);
        buzzer_set_delay(2000, slice_num, true);

        // Desligado
        pwm_set_enabled(slice_num, false);
        buzzer_set_delay(2000, slice_num, true);
      }
    } else {
      // verifica se já se passaram os 3 segundos
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

// Implementa a tarefa dos LEDs
void vLedMatrixTask() {
  // Iniciliza a matriz de LEDs, faz a limpeza do buffer e envia para a matriz
  npInit(LED_PIN);
  npClear();
  npWrite();

  const TickType_t step = pdMS_TO_TICKS(STEP_MS);
  const uint stepsPer = 2000 / STEP_MS;

  while (true) {
    // Verifica se o modo é noturno
    if (!is_night_mode) {
      // verifica se já se passaram os 3 segundos, se o sinal é o verde e quantos pulsos emitidos pela matriz já ocorreram.
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
      // Apaga a matriz de LEDs
      npClear();
      npWrite();
    }
  }
}

// Implementa a tarefa dos LEDs RGB (semaforo)
void vLEDsRGBTask() {
  // Iniciliza os LEDs
  led_rgb_setup(LED_RED);
  led_rgb_setup(LED_GREEN);
  led_rgb_setup(LED_BLUE);

  while (true) {
    // Verifica se o modo é noturno
    if (is_night_mode) {
      // Entrando no modo noturno
      if (counter == 0) { // verifica se counter é igual a zero. Se sim, desliga todos os LEDs e aplica delay de 3 segundos.
        gpio_put(LED_RED, false);
        gpio_put(LED_GREEN, false);
        gpio_put(LED_BLUE, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        counter++;
        // após o delay de 3 segundos e incrementa a variável counter. Para que esse bloco seja executado apenas quando
        // o botão A for pressionado (antes do início do primeiro ciclo).
      }

      printf("contador: %d\n", counter);

      // liga
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, true);
      leds_set_delay(2000, true);

      // deliga
      gpio_put(LED_RED,   false);
      gpio_put(LED_GREEN, false);
      leds_set_delay(2000, true);
    } else {
      // Entrando no modo normal
      if (counter == 0) { // verifica se counter é igual a zero. Se sim, desliga todos os LEDs e aplica delay de 3 segundos.
        gpio_put(LED_RED, false);
        gpio_put(LED_GREEN, false);
        gpio_put(LED_BLUE, false);
        vTaskDelay(pdMS_TO_TICKS(3000));
        counter++;
        // após o delay de 3 segundos e incrementa a variável counter. Para que esse bloco seja executado apenas quando
        // o botão A for pressionado (antes do início do primeiro ciclo).
      }

      printf("contador: %d\n", counter);

      // Verde por 8s
      traffic_light_step = 0; // Define que o sinal verde está ativado (utilizado para definir os estados do buzzer e do display)
      gpio_put(LED_RED,   false);
      gpio_put(LED_GREEN, true);
      leds_set_delay(8000, false);

      // Amarelo por 4s
      traffic_light_step = 1; // Define que o sinal amarelo está ativado (utilizado para definir os estados do buzzer e do display)
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, true);
      leds_set_delay(4000, false);

      // Vermelho por 8s
      traffic_light_step = 2; // Define que o sinal vermelho está ativado (utilizado para definir os estados do buzzer e do display)
      ledsMatrixCounter = 0;
      gpio_put(LED_RED,   true);
      gpio_put(LED_GREEN, false);
      leds_set_delay(8000, false);
    }
  }
}

// Função responsável por tratar as interrupções do botão A e B
void gpio_irq_handler(uint gpio, uint32_t events) {
  uint32_t current_time = to_ms_since_boot(get_absolute_time()); // Vetorna o tempo total em ms desde o boot do rp2040

  // Verifica se a diff entre o tempo atual e a ultima vez que o botão foi pressionado é maior que o tempo de debounce
  if (current_time - last_time_btn_press > debounce_delay_ms) {
    last_time_btn_press = current_time;

    // Tratamento da interrupção do botão A
    if (gpio == BTN_A_PIN) {
      counter = 0; // Zera a variável counter para aplicar o delay de 3 segundos antes do primeiro ciclo do modo de operação definido para o semaforo
      is_night_mode = !is_night_mode; // Altera o modo de operação do semáforo (normal/noturno)
      traffic_light_step = 0;
      ledsMatrixCounter = 0;
      pwm_set_enabled(slice_num, false); // Desliga o buzzer

      // Desliga a matriz de LEDs
      npClear();
      npWrite();

      // Imprime mensagem de DEBUG no monitor serial
      printf("contador: %d\n", counter);

      if (is_night_mode) {
        printf("Modo Atual: Noturno.\n");
      } else {
        printf("Modo Atual: Normal.\n");
      }
    } else if (gpio == BTN_B_PIN) { // Tratamento da interrupção do botão B
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
