/**
 * =============================================================
 * Projeto : Controle de Servo Motor 0°–180° com Joystick
 * Plataforma: Raspberry Pi Pico
 * =============================================================
 *
 * Hardware:
 *  - Joystick Analógico:
 *      VRx  -> GP27 (ADC1) — Eixo Horizontal  ← USADO PARA POSIÇÃO ANGULAR
 *      VCC  -> 3.3V
 *      GND  -> GND
 *
 *  - Servo Motor Padrão (0°–180°):
 *      Sinal -> GP19 (PWM)
 *      VCC   -> VBUS (5V)
 *      GND   -> GND
 *
 * ──────────────────────────────────────────────────────────────
 * LÓGICA DE CONTROLE:
 *
 *  O joystick horizontal controla a POSIÇÃO ANGULAR do servo:
 *
 *   Joystick esquerda → 0°   (servo vai para CIMA - extremidade mínima)
 *   Joystick centro   → 90°  (servo no MEIO)
 *   Joystick direita  → 180° (servo vai para BAIXO - extremidade máxima)
 *
 *  A posição é mantida enquanto o joystick estiver naquele ponto.
 *
 * ──────────────────────────────────────────────────────────────
 * COMO O SERVO 0°–180° FUNCIONA COM PWM:
 *
 *  O servo padrão interpreta a largura do pulso
 *  como POSIÇÃO ANGULAR:
 *
 *   ~500µs  → 0°   (CIMA - extremidade mínima)
 *   ~1500µs → 90°  (MEIO - posição central)
 *   ~2500µs → 180° (BAIXO - extremidade máxima)
 *
 *  OBS: Os valores exatos podem variar por modelo.
 *       Ajuste SERVO_MIN_US e SERVO_MAX_US se necessário.
 * =============================================================
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"   // Periférico ADC — leitura analógica do joystick
#include "hardware/pwm.h"   // Periférico PWM — geração do sinal para o servo

// ══════════════════════════════════════════════
// SEÇÃO 1 — DEFINIÇÕES DE PINOS
// ══════════════════════════════════════════════

#define PIN_VRX      27     // GP27 → ADC1: eixo horizontal do joystick
#define PIN_SERVO    19     // GP19 → PWM:  sinal de controle do servo

// ══════════════════════════════════════════════
// SEÇÃO 2 — PARÂMETROS DO ADC
//
//  O ADC do Pico possui resolução de 12 bits:
//    Fórmula: N_bits = 12  →  2^12 = 4096 valores possíveis
//    Intervalo: 0 (0V) até 4095 (3.3V)
//
//  Mapeamento para ângulo (0° a 180°):
//    ADC = 0    → 0°    (joystick à esquerda - CIMA)
//    ADC = 2048 → 90°   (joystick no centro - MEIO)
//    ADC = 4095 → 180°  (joystick à direita - BAIXO)
//
//  Sem zona morta: cada posição é mantida continuamente
// ══════════════════════════════════════════════

#define ADC_RESOLUCAO    4096     // 2^12 — total de níveis do ADC de 12 bits

// ══════════════════════════════════════════════
// SEÇÃO 3 — PARÂMETROS DO PWM
//
//  O Pico usa um clock interno de 125 MHz.
//  Para gerar um sinal PWM de 50 Hz (período = 20ms):
//
//  PASSO 1 — Escolher o divisor de clock:
//    clock_pwm = 125 MHz / CLK_DIV
//    CLK_DIV = 125  →  clock_pwm = 125.000.000 / 125 = 1.000.000 Hz = 1 MHz
//    Isso significa: cada tick do contador PWM dura 1/1.000.000 s = 1 µs
//
//  PASSO 2 — Calcular o WRAP (contador máximo):
//    periodo_desejado = 20 ms = 20.000 µs
//    WRAP = periodo_desejado / (1/clock_pwm)
//    WRAP = 20.000 µs / 1 µs = 20.000 ticks
//    O contador conta: 0, 1, 2, ..., 19999, 0, 1, 2, ... (reinicia)
//    Frequência real = clock_pwm / WRAP = 1.000.000 / 20.000 = 50 Hz  ✓
//
//  PASSO 3 — Largura de pulso em ticks:
//    Como 1 tick = 1 µs, a largura em ticks é numericamente igual à em µs
//    Ex: pulso de 1500µs → CC (compare level) = 1500 ticks
//        O pino fica HIGH de 0 a 1499, e LOW de 1500 a 19999
// ══════════════════════════════════════════════
//125.0f
#define PWM_CLK_DIV      125.0f   // Divisor: 125 MHz / 125 = 1 MHz (1 tick = 1 µs)
#define PWM_WRAP         20000    // 20.000 ticks × 1µs = 20ms → 50 Hz

// ──────────────────────────────────────────────
// Parâmetros de pulso do servo 0°–180°:
//
//  SERVO_MIN_US   = pulso para 0°   (CIMA - extremidade mínima)
//  SERVO_CENTER_US = pulso para 90° (MEIO - posição central)
//  SERVO_MAX_US   = pulso para 180° (BAIXO - extremidade máxima)
//
//  Intervalo utilizável: [500µs ... 1500µs ... 2500µs]
//  Faixa total: 2000µs (cobrindo os 180° de movimento)
// ──────────────────────────────────────────────

#define SERVO_MIN_US     500     // Pulso mínimo: 0° (CIMA)
#define SERVO_CENTER_US  1500     // Pulso central: 90° (MEIO)
#define SERVO_MAX_US     2500     // Pulso máximo: 180° (BAIXO)
#define SERVO_FAIXA_US   2000     // SERVO_MAX_US - SERVO_MIN_US = 2000µs


uint16_t valor_x = 0;
uint16_t pulso = 0 ;

uint16_t ler_adc(uint canal);
void     set_pulso_servo(uint16_t pulso_us);
uint16_t calcular_pulso(uint16_t adc_raw);


// ──────────────────────────────────────────────

// FUNÇÃO: ler_adc
//
// Seleciona o canal ADC e retorna a leitura de 12 bits.
//   canal 0 → GP26 (ADC0)
//   canal 1 → GP27 (ADC1)  ← usado aqui para o eixo X
//
// Retorna: uint16_t com valor de 0 a 4095
// ──────────────────────────────────────────────
uint16_t ler_adc(uint canal) {
    adc_select_input(canal);    // Conecta o multiplexador interno ao canal escolhido
    return adc_read();          // Dispara a conversão e retorna o resultado de 12 bits
}

// ──────────────────────────────────────────────
// FUNÇÃO: set_pulso_servo
//
// Envia ao servo um pulso de largura `pulso_us` microssegundos.
//
// Internamente, define o Compare Level (CC) do canal PWM:
//   - O pino GP19 fica HIGH enquanto contador < pulso_us
//   - O pino GP19 fica LOW  enquanto contador >= pulso_us até WRAP
//   - Isso gera um pulso de duração exata de pulso_us µs a cada 20ms
// ──────────────────────────────────────────────
void set_pulso_servo(uint16_t pulso_us) {
    uint slice   = pwm_gpio_to_slice_num(PIN_SERVO);
    uint canal   = pwm_gpio_to_channel(PIN_SERVO);  // Canal B do Slice 1

    // pwm_set_chan_level define o valor de comparação CC:
    //   contador [0 .. pulso_us-1]  → saída HIGH
    //   contador [pulso_us .. WRAP] → saída LOW
    pwm_set_chan_level(slice, canal, pulso_us);
}

// ──────────────────────────────────────────────
// FUNÇÃO: calcular_pulso
//
// Converte a leitura bruta do ADC em largura de pulso PWM para posição angular.
//
// CÁLCULO COMPLETO (passo a passo):
//
//  Entrada: adc_raw  → valor lido do ADC (0 a 4095)
//
//  PASSO 1 — Normalizar para proporção [0.0 ... 1.0]:
//    proporcao = adc_raw / 4095
//    proporcao = 0.0   → joystick na esquerda (será 180°)
//    proporcao = 0.5   → joystick no centro (será 0°)
//    proporcao = 1.0   → joystick na direita (será -180°)
//
//  PASSO 2 — Inverter a proporção (para mapear corretamente):
//    proporcao_invertida = 1.0 - proporcao
//
//  PASSO 3 — Mapear proporção invertida para variação de pulso:
//    variacao_us = proporcao_invertida × SERVO_FAIXA_US
//    SERVO_FAIXA_US = 2000µs
//
//  PASSO 4 — Calcular pulso final somando ao mínimo:
//    pulso_us = SERVO_MIN_US + variacao_us
//
//  PASSO 5 — Saturação (clamping):
//    Garante que o pulso fique dentro de [SERVO_MIN_US, SERVO_MAX_US]
//
// Retorna: largura de pulso em µs pronta para enviar ao PWM
// ──────────────────────────────────────────────
uint16_t calcular_pulso(uint16_t adc_raw) {

    // PASSO 1: Normalização — converte o valor inteiro em proporção float
    //   proporcao = adc_raw / 4095
    //   Resultado: valor entre 0.0 e 1.0
    float proporcao = (float)adc_raw / (float)(ADC_RESOLUCAO - 1);

    // Satura a proporção em [0.0, 1.0] por segurança
    if (proporcao >  1.0f) proporcao = 1.0f;
    if (proporcao <  0.0f) proporcao = 0.0f;

    // PASSO 2: Inverter a proporção para mapear corretamente
    //   joystick esquerda (adc=0, prop=0.0) → inverted=1.0 → 180°
    //   joystick centro (adc=2048, prop=0.5) → inverted=0.5 → 0°
    //   joystick direita (adc=4095, prop=1.0) → inverted=0.0 → -180°
    float proporcao_invertida = 1.0f - proporcao;

    // PASSO 3: Variação de pulso — escala a proporção invertida pela faixa do servo
    //   variacao_us = proporcao_invertida × 2000µs
    float variacao_us = proporcao_invertida * (float)SERVO_FAIXA_US;

    // PASSO 4: Pulso final — soma ao mínimo
    //   pulso = 500µs + variacao_us
    float pulso_f = (float)SERVO_MIN_US + variacao_us;

    // PASSO 5: Saturação do pulso no intervalo permitido [500µs, 2500µs]
    if (pulso_f > (float)SERVO_MAX_US) pulso_f = (float)SERVO_MAX_US;
    if (pulso_f < (float)SERVO_MIN_US) pulso_f = (float)SERVO_MIN_US;

    return (uint16_t)pulso_f;
}


int main(void) {
    // Inicializa a comunicação serial USB (para ver os logs no monitor serial)
    stdio_init_all();

    adc_init();
    adc_gpio_init(PIN_VRX);   // Configura ADC1 no GP27 para leitura do joystick

    // FUNÇÃO: setup_pwm
//
// Configura o PWM no GP19:
//   - Define GP19 como saída de função PWM
//   - Aplica o divisor de clock (125 → 1 MHz)
//   - Define o WRAP (20000 → período de 20ms)
//   - Inicializa o servo no pulso neutro (PARADO)
//   - Habilita o slice PWM
// ──────────────────────────────────────────────
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);

    // O Pico tem 8 slices PWM, cada um com 2 canais (A e B).
    // A função abaixo descobre qual slice está ligado ao GP19.
    // GP19 = Slice 1, Canal B  (conforme datasheet RP2040, tabela de pinos)
    uint slice = pwm_gpio_to_slice_num(PIN_SERVO);

   
    pwm_set_clkdiv(slice, PWM_CLK_DIV);

    // Define o valor máximo do contador (WRAP):
    //   O contador incrementa: 0, 1, 2, ..., 19999, [reset para 0]
    //   Período = 20.000 ticks × 1µs/tick = 20.000 µs = 20 ms → 50 Hz
    pwm_set_wrap(slice, PWM_WRAP);

    // Inicia com servo na posição central (90°, pulso de 1500µs)
    set_pulso_servo(SERVO_CENTER_US);

    // Habilita o slice
    pwm_set_enabled(slice, true); 


    while (true) {

        // ── ETAPA 1: Leitura do ADC ───────────────────────────────
        // Lê o canal ADC1 (GP27 = eixo X do joystick)
        // Retorna valor de 12 bits: 0 a 4095
        valor_x = ler_adc(1);//passa o canal do adc

        // ── ETAPA 2: Cálculo direto do pulso PWM ─────────────────
        // Mapeia 0-4095 → 0°-180° → 500-2500µs
        // A função interna faz: proporcao, variação, pulso e saturação
        pulso = calcular_pulso(valor_x);

        // ── ETAPA 3: Envio do pulso ao servo via PWM ─────────────
        // Define o Compare Level do canal PWM: pino HIGH por `pulso` ticks (= µs)
        set_pulso_servo(pulso);

        // ── ETAPA 4: Exibição dos cálculos no serial ─────────────
        

        // Aguarda 20ms (sincronizado com o período do PWM)
        // O servo recebe um pulso novo a cada ciclo de 20ms
        sleep_ms(20);
    }

    return 0;
}