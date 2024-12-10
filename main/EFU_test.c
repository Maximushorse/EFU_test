
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


//Voltage converter drivers enable setup
#define ENABLE_PIN_A 8  // enable A
#define ENABLE_PIN_B 37 // enable B
int enable_state_A = 1;   // ================= ustawienie enable (1 on, 0 off)
int enable_state_B = 1;  // ================= ustawienie enable (1 on, 0 off)

//Voltage converter drivers PWM signal setup
#define PWM_PIN_A 7  // PWM A Buck/current control
#define PWM_PIN_B 36 // PWM B Boost 

// Drivers PWM start filling (%)
int PWM_duty_cycle_percent_A = 10; // PWM A Buck/current control
int PWM_duty_cycle_percent_B = 100; // PWM B Boost - reverse to voltage: min voltage = 100, max voltage = 40


//Average
int SC_numb = 0;
int SC_V_av[11];
int SC_V = 0;

int FC_numb = 0;
int FC_C_av[21];
int FC_C = 0;

int adc_raw[2][10];
int adc_cal[2][10];

// frequncy of clock (each tick)
// 2.5 MHz -> 40 ns
#define SERVO_TIMEBASE_RESOLUTION_HZ 2500000

void converter();
void boost_converter ();

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

void pwm_configuration() {
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

float mcpwm_duty_cycle_calculate(float duty_cycle_percent) {
  return ((duty_cycle_percent / 100) * SERVO_TIMEBASE_PERIOD);
}

void enable_configuration() {
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

const static char *TAG = "EXAMPLE";
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                         adc_atten_t atten,
                                         adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_FAIL;
  bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif

  *out_handle = handle;
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Calibration Success");
  } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
    ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
  } else {
    ESP_LOGE(TAG, "Invalid arg or no memory");
  }

  return calibrated;
}

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_handle_t adc2_handle;
adc_cali_handle_t cali1 = NULL;
adc_cali_handle_t cali2 = NULL;
adc_cali_handle_t cali3 = NULL;
adc_cali_handle_t cali4 = NULL;
adc_cali_handle_t cali5 = NULL;

adc_cali_handle_t cali6 = NULL;
adc_cali_handle_t cali7 = NULL;

void adc_init() {
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };

  adc_oneshot_unit_init_cfg_t init_config2 = {
      .unit_id = ADC_UNIT_2,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  adc_oneshot_new_unit(&init_config2, &adc2_handle);

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_6,
      .bitwidth = ADC_BITWIDTH_12,
  };

  //   adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config);
  //   adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config);
  //   adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_8, &config);

  //   adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_6, &config);
  //   adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config);


  example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_2, ADC_ATTEN_DB_6, //SC_V 0-55V to 0-3V
                               &cali1);
  example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, ADC_ATTEN_DB_6, //FC_V 0-55V to 0-3V
                               &cali2);
  example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_8, ADC_ATTEN_DB_6, //FC_C +-10A to 0,5-3V 0=1,5V
                               &cali3);
  example_adc_calibration_init(ADC_UNIT_2, ADC_CHANNEL_6, ADC_ATTEN_DB_6, //MC_C +-30A to 0,5-3V 0=1,5V
                               &cali4);
  example_adc_calibration_init(ADC_UNIT_2, ADC_CHANNEL_7, ADC_ATTEN_DB_6, //SC_C +-30A to 0,5-3V 0=1,5V
                               &cali5);

    example_adc_calibration_init(ADC_UNIT_2, ADC_CHANNEL_0, ADC_ATTEN_DB_6, //Aditional Voltage mesurment
                               &cali6);
    example_adc_calibration_init(ADC_UNIT_2, ADC_CHANNEL_4, ADC_ATTEN_DB_6, //Aditional Voltage mesurment
                               &cali7);
}


//Logs sending using FreeRTOS xTask

// Creating logs and send
void send_esp_logi() {
    ESP_LOGI("ADC",
             "CAL   SC_V: %d,   FC_V: %d,   FC_C: %d,   SC_C: %d,   MC_C: %d,  33_V: %d,  12_V: %d, \tPWM_B: %d\tSC_Vav: %d,\tFC_C: %d",
             adc_cal[1][2], adc_cal[1][4], adc_cal[1][8], adc_cal[2][6],
             adc_cal[2][7], adc_cal[2][0], adc_cal[2][4], PWM_duty_cycle_percent_B, SC_V, FC_C);
}

// Sending logs
void log_task(void *pvParameters) {
    while (1) {
        send_esp_logi();                 
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait 1000ms to next 
    }
}




void app_main(void) {
  //FreeRTOS task working separetly - 4096 is the memory slot
  xTaskCreate(log_task, "Log Task", 4096, NULL, 1, NULL);

 // esp_task_wdt_delete(NULL); // Turn Off watchdog (min 10ms vTaskDelay in main)


  enable_configuration();
  pwm_configuration();
  adc_init();

  // Voltage converter drivers enable change
  gpio_set_level(ENABLE_PIN_A, enable_state_A);
  gpio_set_level(ENABLE_PIN_B, enable_state_B);

  // Voltage converter PWM A (Buck/Current regulator) duty cycle change
  mcpwm_comparator_set_compare_value(
      comparator_handle_1, mcpwm_duty_cycle_calculate(PWM_duty_cycle_percent_A));

  // Voltage converter PWM B (Boost) duty cycle change
  mcpwm_comparator_set_compare_value(
      comparator_handle_2, mcpwm_duty_cycle_calculate(PWM_duty_cycle_percent_B));






  while (1) {       //ADC selection, channel,    tabell int value set
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw[1][2]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[1][4]);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_8, &adc_raw[1][8]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &adc_raw[2][6]); // [1][6]
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &adc_raw[2][7]);

    adc_oneshot_read(adc2_handle, ADC_CHANNEL_0, &adc_raw[2][0]);
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_4, &adc_raw[2][4]);
    // ESP_LOGI("ADC",
    //          "RAW \tSC_V: %d, \tFC_V: %d, \tFC_C: %d, \tSC_V: %d, \tMC_C:
    //          %d", adc_raw[1][0], adc_raw[0][4], adc_raw[0][8], adc_raw[1][6],
    //          adc_raw[2][7]);

                            //mV calculation using integers
    adc_cali_raw_to_voltage(cali1, adc_raw[1][2], &adc_cal[1][2]); //SC_V 0-55V to 0-3V
    adc_cali_raw_to_voltage(cali1, adc_raw[1][4], &adc_cal[1][4]);//FC_V 0-55V to 0-3V
    adc_cali_raw_to_voltage(cali1, adc_raw[1][8], &adc_cal[1][8]); //FC_C +-10A to 0,5-3V 0=1,5V
    adc_cali_raw_to_voltage(cali1, adc_raw[2][6], &adc_cal[2][6]); //MC_C +-30A to 0,5-3V 0=1,5V
    adc_cali_raw_to_voltage(cali1, adc_raw[2][7], &adc_cal[2][7]); //SC_C +-30A to 0,5-3V 0=1,5V

    adc_cali_raw_to_voltage(cali1, adc_raw[2][0], &adc_cal[2][0]); //Aditional Voltage mesurment 3.3V
    adc_cali_raw_to_voltage(cali1, adc_raw[2][4], &adc_cal[2][4]); //Aditional Voltage mesurment 12V


      

      
    //Średnia ruchoma - przerobić na funkcje
    if (SC_numb <= 10){
      SC_V_av[SC_numb] = adc_cal[1][0];
      SC_numb++;
    } else {
        SC_numb=0;
    }
    for(int i=0; i<10; i++){
          SC_V = SC_V + SC_V_av[i];
        }
      SC_V = SC_V/10;


    if (FC_numb <= 20){
      FC_C_av[FC_numb] = adc_cal[1][8];
      FC_numb++;
    } else {
        FC_numb=0;
    }
    for(int j=0; j<10; j++){
          FC_C = FC_C + FC_C_av[j];
        }
      FC_C = FC_C/20 - 180; //TEST...



    //converter();
    // enable_state_A = 1;
    // enable_state_B = 1;
    // PWM_duty_cycle_percent_B = 50;
    // PWM_duty_cycle_percent_A = 100;
    boost_converter ();

    //Safty
    if(PWM_duty_cycle_percent_A <= 35) PWM_duty_cycle_percent_A = 35; //Buck D min = 35% (min 20V SC_V)
    if(PWM_duty_cycle_percent_B >= 62) PWM_duty_cycle_percent_B = 62; //Bost D max = 62% (max 60V)  50V -> 45%

    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}


void boost_converter (){
  int SC_V_SET = 2509;//To read from CAN comunication 3000mV = 55V 46V->2509
  int FC_C_MAX = 1650;  // 3000mV = 10A lub -10A      3000/20=150   -10A->150   -1A->1500-150=1350  1A->1650
   
  if(FC_C <= FC_C_MAX) {
    enable_state_A = 1;
    enable_state_B = 1;
    PWM_duty_cycle_percent_A = 100;

    if(SC_V < SC_V_SET) {
      PWM_duty_cycle_percent_B+=1;
    } else {
      PWM_duty_cycle_percent_B-=5;
    }
  } else {
      if(FC_C > FC_C_MAX*2) {
      enable_state_A = 1;
      enable_state_B = 1;
      }
    PWM_duty_cycle_percent_A = 100;
    PWM_duty_cycle_percent_B-=10;
  }

}


//DOCELOWE MUSI OPIERAĆ SIĘ O STABILIZACJĘ PRĄDU POBIERANEGO Z OGNIWA I NAPIĘCIE MAKSYNALNE SC

//First converter control function - simple step up
void converter (){
  int SC_V_MAX = 2509; //To read from CAN comunication 3000mV = 55V 46V->2509
  int FC_C_SET = 1650;  // 3000mV = 10A lub -10A      3000/20=150   -10A->150   -1A->1500-150=1350  1A->1650
                      //TEST HALL Direction!!!

  if (SC_V <= SC_V_MAX && SC_V > 2000)
  {
      enable_state_A = 1;
      enable_state_B = 1;

      if (FC_C < FC_C_SET)
      {
        PWM_duty_cycle_percent_B --; //soft start
      }

      if (FC_C > FC_C_SET && FC_C < FC_C_SET*2) //Percent of max current - read fron CAB communication
      {
        PWM_duty_cycle_percent_B = PWM_duty_cycle_percent_B + 5; //Simple value or proportional to current
      }

      if(FC_C >= FC_C_SET*2)
      {
        enable_state_A = 0; //Maby use PWM A to buck conversion and current contol
        PWM_duty_cycle_percent_B = 1; 
      }

      //TODO  -- Add limiters for sefty  NPT Resistor for 0V SC start?
  } else
  {
    enable_state_A = 0;
    enable_state_B = 0;

    //full charged SC flag
    bool SC_FULL = 1;
  }
}



