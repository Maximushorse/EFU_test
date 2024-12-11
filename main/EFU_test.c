
#include <math.h>
#include <stdio.h>

#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>

#include "driver/gpio.h"

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/mcpwm_types.h"
#include "esp_task_wdt.h"

// Voltage converter drivers enable setup
#define ENABLE_PIN_A 8  // enable A
#define ENABLE_PIN_B 37 // enable B
int enable_state_A = 0; // ================= ustawienie enable (1 on, 0 off)
int enable_state_B = 0; // ================= ustawienie enable (1 on, 0 off)

// Voltage converter drivers PWM signal setup
#define PWM_PIN_A 7  // PWM A Buck/current control
#define PWM_PIN_B 36 // PWM B Boost

// Drivers PWM start filling (%)
float PWM_duty_cycle_percent_A = 100;  // PWM A Buck/current control
float PWM_duty_cycle_percent_B = 100; // PWM B Boost - reverse to voltage: min voltage = 100, max voltage = 40

float adc_raw_to_mv_calibrated(int adc_raw);

// Average
int SC_V_numb = 0;
float SC_V_av[21];
float SC_V = 0;

int SC_C_numb = 0;
float SC_C_av[21];
float SC_C = 0;

int FC_C_numb = 0;
float FC_C_av[21];
float FC_C = 0;

int FC_V_numb = 0;
float FC_V_av[21];
float FC_V = 0;

uint16_t adc_raw[2][10];
float adc_cal[2][10];

// frequncy of clock (each tick)
// 2.5 MHz -> 40 ns
#define SERVO_TIMEBASE_RESOLUTION_HZ 2000000

void converter();
void boost_converter();

// number of ticks in each period
// 100 ticks, 40 us;
#define SERVO_TIMEBASE_PERIOD 100

mcpwm_timer_handle_t timer_handle_1 = NULL;
mcpwm_oper_handle_t operator_handle_1 = NULL;
mcpwm_cmpr_handle_t comparator_handle_1 = NULL;
mcpwm_gen_handle_t generator_handle_1 = NULL;
mcpwm_timer_handle_t timer_handle_2 = NULL;
mcpwm_oper_handle_t operator_handle_2 = NULL;
mcpwm_cmpr_handle_t comparator_handle_2 = NULL;
mcpwm_gen_handle_t generator_handle_2 = NULL;

void pwm_configuration()
{
  // timer 1
  mcpwm_timer_config_t timer_config1 = {
      .group_id = 0,
      .intr_priority = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
  };
  mcpwm_new_timer(&timer_config1, &timer_handle_1);

  // timer 2
  mcpwm_timer_config_t timer_config2 = {
      .group_id = 0,
      .intr_priority = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = SERVO_TIMEBASE_PERIOD,
  };
  mcpwm_new_timer(&timer_config2, &timer_handle_2);

  // operator 1
  mcpwm_operator_config_t operator_config1 = {
      .group_id = 0, // operator must be in the same group to the timer
  };
  mcpwm_new_operator(&operator_config1, &operator_handle_1);

  // operator 2
  mcpwm_operator_config_t operator_config2 = {
      .group_id = 0, // operator must be in the same group to the timer
  };
  mcpwm_new_operator(&operator_config2, &operator_handle_2);

  // comparator 1
  mcpwm_comparator_config_t comparator_config1 = {
      .flags.update_cmp_on_tez = true,
  };
  mcpwm_new_comparator(operator_handle_1, &comparator_config1,
                       &comparator_handle_1);

  // comparator 2
  mcpwm_comparator_config_t comparator_config2 = {
      .flags.update_cmp_on_tez = true,
  };
  mcpwm_new_comparator(operator_handle_2, &comparator_config2,
                       &comparator_handle_2);

  // generator 1
  mcpwm_generator_config_t generator_config1 = {
      .gen_gpio_num = PWM_PIN_A,
  };
  mcpwm_new_generator(operator_handle_1, &generator_config1,
                      &generator_handle_1);

  // generator 2
  mcpwm_generator_config_t generator_config2 = {
      .gen_gpio_num = PWM_PIN_B,
  };
  mcpwm_new_generator(operator_handle_2, &generator_config2,
                      &generator_handle_2);

  // operator 1 and generator 1 connection
  mcpwm_operator_connect_timer(operator_handle_1, timer_handle_1);

  // operator 2 and generator 2 connection
  mcpwm_operator_connect_timer(operator_handle_2, timer_handle_2);

  // other configs 1
  mcpwm_generator_set_action_on_timer_event(
      generator_handle_1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH));
  mcpwm_generator_set_action_on_compare_event(
      generator_handle_1,
      MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP, comparator_handle_1, MCPWM_GEN_ACTION_LOW));
  mcpwm_timer_enable(timer_handle_1);
  mcpwm_timer_start_stop(timer_handle_1, MCPWM_TIMER_START_NO_STOP);

  // other configs 2
  mcpwm_generator_set_action_on_timer_event(
      generator_handle_2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                       MCPWM_TIMER_EVENT_EMPTY,
                                                       MCPWM_GEN_ACTION_HIGH));
  mcpwm_generator_set_action_on_compare_event(
      generator_handle_2,
      MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP, comparator_handle_2, MCPWM_GEN_ACTION_LOW));
  mcpwm_timer_enable(timer_handle_2);
  mcpwm_timer_start_stop(timer_handle_2, MCPWM_TIMER_START_NO_STOP);
}

float mcpwm_duty_cycle_calculate(float duty_cycle_percent)
{
  return ((duty_cycle_percent / 100) * SERVO_TIMEBASE_PERIOD);
}

void enable_configuration()
{
  gpio_config_t gpio_config1 = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << ENABLE_PIN_A),
  };
  gpio_config(&gpio_config1);

  gpio_config_t gpio_config2 = {
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1ULL << ENABLE_PIN_B),
  };
  gpio_config(&gpio_config2);
}

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;

void adc_init()
{
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };

  adc_oneshot_unit_init_cfg_t init_config2 = {
      .unit_id = ADC_UNIT_2,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  adc_oneshot_new_unit(&init_config2, &adc2_handle);

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };

  adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config);
  adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config);
  adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_8, &config);

  adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_0, &config);
  adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_2, &config);
  adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_6, &config);
  adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config);
}

// Logs sending using FreeRTOS xTask

// Creating logs and send
void send_esp_logi()
{
  printf("%2.3f," 
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%2.3f,"
         "%3.2f,"
         "%3.2f\n\r",
         adc_cal[1][2], adc_cal[1][4], adc_cal[1][8], adc_cal[2][6],
         adc_cal[2][7], adc_cal[2][0], adc_cal[2][2], SC_V, FC_C, FC_V, SC_C, PWM_duty_cycle_percent_A, PWM_duty_cycle_percent_B);
}

// Sending logs
void log_task(void *pvParameters)
{
  while (1)
  {
    send_esp_logi();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1000ms to next
  }
}

float adc_raw_to_mv_calibrated(int adc_raw)
{
  // dac swiezakowi jakiemus (minster cyfryzacji prof. nadzwyczajny dr mgr edukacji wczesnoszkolnej inż. Marek K. (podkarpacianianin)(pomnik czynu rewolucyjnego)) z multimetrem zeby spisal adc_raw i napiecie z woltomierza i ulozyc nowe wspolczynniki dla wielomianu
  // na 1 roku jest to to sobie przecwiczy
  // najlepiej tak ze jak juz bedzie szlo do auta albo bedziemy potrzebowac dokladncyh wynikow
  // kalibracje trzeba dla kazdej plytki zrobic ale chyba nie dla adc1 i adc2 oddzielnie

  float adc_raw_f = adc_raw;
  //adc_raw_f = adc_raw_f;

  float adc_mV = 0;

  
  float a[11];
  a[0] =      2.5679088051711859e-004;
  a[1] =      8.1818138383964200e-004;
  a[2] =      4.4561346617176653e-007;
  a[3] =     -1.5785641837443660e-009;
  a[4] =      2.6100902814800432e-012;
  a[5] =     -2.4247998157041470e-015;
  a[6] =      1.3522880124391428e-018;
  a[7] =     -4.6201730839219227e-022;
  a[8] =      9.4673093903828406e-026;
  a[9] =     -1.0679135163074677e-029;
  a[10] =     5.0946016674492631e-034;

  for (int i = 0; i < 11; i++)
  {
    for (int j = 0; j < i; j++)
    {
      a[i] *= adc_raw_f;
    }
    adc_mV += a[i];
  }

  return adc_mV;
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void app_main(void)
{
  // FreeRTOS task working separetly - 4096 is the memory slot
  xTaskCreate(log_task, "Log Task", 4096, NULL, 1, NULL);

  // esp_task_wdt_delete(NULL); // Turn Off watchdog (min 10ms vTaskDelay in main)

  enable_configuration();
  pwm_configuration();
  adc_init();



  while (1)
  { // ADC selection, channel,    tabell int value set
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[1][2]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[1][4]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_8, &adc_raw[1][8]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &adc_raw[2][6]); // [1][6]
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw[2][7]);

    adc_oneshot_read(adc2_handle, ADC_CHANNEL_0, &adc_raw[2][0]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_2, &adc_raw[2][2]);
    // ESP_LOGI("ADC",
    //          "RAW \tSC_V: %d, \tFC_V: %d, \tFC_C: %d, \tSC_V: %d, \tMC_C:
    //          %d", adc_raw[1][0], adc_raw[0][4], adc_raw[0][8], adc_raw[1][6],
    //          adc_raw[2][7]);
    
    //Cealn from error values
    if (adc_raw[1][2] > 4095) adc_raw[1][2] = 0;
    if (adc_raw[1][4] > 4095) adc_raw[1][4] = 0;
    if (adc_raw[1][8] > 4095) adc_raw[1][8] = 0;
    if (adc_raw[2][6] > 4095) adc_raw[2][6] = 0;
    if (adc_raw[2][7] > 4095) adc_raw[2][7] = 0;
    if (adc_raw[2][0] > 4095) adc_raw[2][0] = 0;
    if (adc_raw[2][2] > 4095) adc_raw[2][2] = 0;

    if (adc_raw[1][2] < 0) adc_raw[1][2] = 0;
    if (adc_raw[1][4] < 0) adc_raw[1][4] = 0;
    if (adc_raw[1][8] < 0) adc_raw[1][8] = 0;
    if (adc_raw[2][6] < 0) adc_raw[2][6] = 0;
    if (adc_raw[2][7] < 0) adc_raw[2][7] = 0;
    if (adc_raw[2][0] < 0) adc_raw[2][0] = 0;
    if (adc_raw[2][2] < 0) adc_raw[2][2] = 0;

    adc_cal[1][2] = adc_raw_to_mv_calibrated(adc_raw[1][2]); //SC_V 0-55V to 0-3V
    adc_cal[1][4] = adc_raw_to_mv_calibrated(adc_raw[1][4]); //FC_V 0-55V to 0-3V
    adc_cal[1][8] = adc_raw_to_mv_calibrated(adc_raw[1][8]); //FC_C +-10A to 0,5-3V 0=1,5V
    adc_cal[2][6] = adc_raw_to_mv_calibrated(adc_raw[2][6]); //SC_C +-30A to 0,5-3V 0=1,5V
    adc_cal[2][7] = adc_raw_to_mv_calibrated(adc_raw[2][7]); //MC_C +-30A to 0,5-3V 0=1,5V
    adc_cal[2][0] = adc_raw_to_mv_calibrated(adc_raw[2][0]); //Aditional Voltage mesurment 3,3V
    adc_cal[2][2] = adc_raw_to_mv_calibrated(adc_raw[2][2]); //Aditional Voltage mesurment 12V

    adc_cal[1][2] = map(adc_cal[1][2], 0, 2.995, 0, 55); //SC_V 0-55V to 0-3V
    adc_cal[1][4] = map(adc_cal[1][4], 0, 2.995, 0, 55); //FC_V 0-55V to 0-3V

    adc_cal[1][8] = map(adc_cal[1][8], 1.5, 1.370, 0, 1); //FC_C +-10A to 0,5-3V 0=1,5V



    adc_cal[2][6] = map(adc_cal[2][6], 1.5, 1.450, 0, 1); //SC_C +-30A to 0,5-3V 0=1,5V

     if(adc_cal[2][6]<=1.502){
        adc_cal[2][6] = map(adc_cal[2][6], 0.5, 1.502, -30, 0); //MC_C +-30A to 0,5-3V 0=1,5V
    } else {
      adc_cal[2][6] = map(adc_cal[2][6], 1.502, 3, 0, 30); //MC_C +-30A to 0,5-3V 0=1,5V
    }



    if(adc_cal[2][7]<=1.502){
        adc_cal[2][7] = map(adc_cal[2][7], 0.5, 1.502, -30, 0); //MC_C +-30A to 0,5-3V 0=1,5V
    } else {
      adc_cal[2][7] = map(adc_cal[2][7], 1.502, 3, 0, 30); //MC_C +-30A to 0,5-3V 0=1,5V
    }
    

    adc_cal[2][0] = map(adc_cal[2][0], 0, 3, 0, 12); //Aditional Voltage mesurment 3,3V
    adc_cal[2][2] = map(adc_cal[2][2], 0, 3, 0, 12); //Aditional Voltage mesurment 12V
    
    
    // Średnia ruchoma - przerobić na funkcje
    if (SC_V_numb <= 20)
    {
      SC_V_av[SC_V_numb] = adc_cal[1][2];
      SC_V_numb++;
    }
    else
    {
      SC_V_numb = 0;
    }
    for (int i = 0; i < 20; i++)
    {
      SC_V = SC_V + SC_V_av[i];
    }
    SC_V = SC_V / 20;



    if (FC_C_numb <= 20)
    {
      FC_C_av[FC_C_numb] = adc_cal[1][8];
      FC_C_numb++;
    }
    else
    {
      FC_C_numb = 0;
    }
    for (int j = 0; j < 20; j++)
    {
      FC_C = FC_C + FC_C_av[j];
    }
    FC_C = FC_C / 20; 




        if (SC_C_numb <= 20)
    {
      SC_C_av[SC_V_numb] = adc_cal[2][6];
      SC_C_numb++;
    }
    else
    {
      SC_C_numb = 0;
    }
    for (int j = 0; j < 20; j++)
    {
      SC_C = SC_C + SC_C_av[j];
    }
    SC_C = SC_C / 20; 



        if (FC_V_numb <= 20)
    {
      FC_V_av[FC_V_numb] = adc_cal[1][4];
      FC_V_numb++;
    }
    else
    {
      FC_V_numb = 0;
    }
    for (int j = 0; j < 20; j++)
    {
      FC_V = FC_V + FC_V_av[j];
    }
    FC_V = FC_V / 20; 








    /* enable_state_A = 0;
    enable_state_B = 0;
    PWM_duty_cycle_percent_B = 0;
    PWM_duty_cycle_percent_A = 0; */

    //converter();
    
    //boost_converter();

    // Safety
    if (PWM_duty_cycle_percent_A <= 35)
      PWM_duty_cycle_percent_A = 35; // Buck D min = 35% (min 20V SC_V)
    if (PWM_duty_cycle_percent_B >= 62)
      PWM_duty_cycle_percent_B = 62; // Bost D max = 62% (max 60V)  50V -> 45%


  // Voltage converter drivers enable change
  gpio_set_level(ENABLE_PIN_A, enable_state_A);
  gpio_set_level(ENABLE_PIN_B, enable_state_B);

  // Voltage converter PWM A (Buck/Current regulator) duty cycle change
  mcpwm_comparator_set_compare_value(
      comparator_handle_1, mcpwm_duty_cycle_calculate(PWM_duty_cycle_percent_A));

  // Voltage converter PWM B (Boost) duty cycle change
  mcpwm_comparator_set_compare_value(
      comparator_handle_2, mcpwm_duty_cycle_calculate(PWM_duty_cycle_percent_B));


    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void boost_converter()
{
  float SC_V_SET = 45; // To read from CAN comunication 3000mV ADC = 55V FC/SC
  float FC_C_MAX = 2; 

  if (FC_C <= FC_C_MAX)
  {
    enable_state_A = 1;
    enable_state_B = 1;
    PWM_duty_cycle_percent_A = 100;

    if (SC_V < SC_V_SET)
    {
      PWM_duty_cycle_percent_B += 1;
    }
    else
    {
      PWM_duty_cycle_percent_B -= 5;
    }
  }
  else
  {
    if (FC_C > FC_C_MAX * 2)
    {
      enable_state_A = 1;
      enable_state_B = 1;
    }
    PWM_duty_cycle_percent_A = 100;
    PWM_duty_cycle_percent_B -= 10;
  }
}

// DOCELOWE MUSI OPIERAĆ SIĘ O STABILIZACJĘ PRĄDU POBIERANEGO Z OGNIWA I NAPIĘCIE MAKSYMALNE SC //poprawiono ortografie

// First converter control function - simple step up
void converter()
{
  int SC_V_MAX = 48; // To read from CAN comunication 3000mV = 55V 46V->2509
  int FC_C_SET = 1; // 3000mV = 10A lub -10A      3000/20=150   -10A->150   -1A->1500-150=1350  1A->1650
                       // TEST HALL Direction!!!

  if (SC_V <= SC_V_MAX && SC_V > 20)
  {
    enable_state_A = 1;
    enable_state_B = 1;

    if (FC_C < FC_C_SET)
    {
      PWM_duty_cycle_percent_B--; // soft start
    }

    if (FC_C > FC_C_SET && FC_C < FC_C_SET * 2) // Percent of max current - read fron CAB communication
    {
      PWM_duty_cycle_percent_B = PWM_duty_cycle_percent_B + 5; // Simple value or proportional to current
    }

    if (FC_C >= FC_C_SET * 2)
    {
      enable_state_A = 0; // Maby use PWM A to buck conversion and current contol
      PWM_duty_cycle_percent_B = 1;
    }

    // TODO  -- Add limiters for sefty  NPT Resistor for 0V SC start?
  }
  else
  {
    enable_state_A = 0;
    enable_state_B = 0;

    // full charged SC flag
    bool SC_FULL = 1;
  }
}
