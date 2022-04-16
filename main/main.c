#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "methodsJS.h"
#include "driver/ledc.h"

//#define DEFAULT_VREF 1100   //ESP32 ADC reference voltage is 1.1V
#define DEFAULT_VREF 1094

//ADC VARIABLES
static const adc_channel_t channelTAmb = ADC_CHANNEL_2;     //GPIO24 ADC2
static const adc_channel_t channelThermocouple = ADC_CHANNEL_0 ;     //GPIO26 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; //12 bits
static const adc_atten_t atten = ADC_ATTEN_DB_11;       //Range of measurementup to  approx. 2600 mV
static const adc_unit_t unit = ADC_UNIT_2;              //ADC2

static esp_adc_cal_characteristics_t *adc_chars;
#define NO_OF_SAMPLES   50          //Multisampling
#define GAIN_INA122P    145

//GLOBAL VARIABLES
uint32_t thermocouple_uV = 0,  adc_reading_Thermocouple_raw = 0;     //_raw used as a control bit if it is 2 or less thermocuple_temp = 0ºC
uint16_t thermocouple_temp = 0, ambient_temp = 0;
uint16_t reference_temperature;

//GPIO define

#define Buzzer_GPIO    GPIO_NUM_19  //Buzzer
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<Buzzer_GPIO))
//GPIO33:  input, pulled up, interrupt from rising edge.
#define Button_GPIO     GPIO_NUM_18  //Button
#define GPIO_INPUT_PIN_SEL  ((1ULL<<Button_GPIO))
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;

//LEDC define
#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LED_GPIO 			   GPIO_NUM_21
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif
#define LEDC_DUTY         	   (4000)
#define LEDC_FADE_TIME    	   (2000)


uint8_t enable = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(io_num == 18){
            	enable = !enable;
            }
        }
    }
}


void get_ADC_values(void *pvParameters)
{
    /* Task that acquires n samples of both the ambient and thermocouple sensors and
       filters them applying an exponential moving average. This task provides
       new values every 250ms */

    //Variables for updating the task exactly every 250ms
    uint8_t cnt = 0;
    const TickType_t xDelay250 = pdMS_TO_TICKS(250);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    //Configure ADC
    //adc2_config_width(width);
    adc2_config_channel_atten(channelTAmb, atten);
    adc2_config_channel_atten(channelThermocouple, atten);
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    int *raw_p1, *raw_p2;
    int raw_tamb = 0, raw_tk = 0;
    raw_p1 = &raw_tamb;
    raw_p2 = &raw_tk;
    while (1)
    {
        uint32_t adc_reading_Tamb = 0, adc_reading_Thermocouple = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc2_get_raw((adc2_channel_t)channelTAmb,width,raw_p1);
            adc_reading_Tamb += *raw_p1;
            adc2_get_raw((adc2_channel_t)channelThermocouple,width,raw_p2);
            adc_reading_Thermocouple += *raw_p2;
        }
        adc_reading_Tamb /= NO_OF_SAMPLES;
        adc_reading_Thermocouple_raw = adc_reading_Thermocouple / NO_OF_SAMPLES;
        cnt++;
        //Convert adc_reading to voltage in mV
        //Now the ambient temperature is obtained.
        uint32_t voltage_ambient = esp_adc_cal_raw_to_voltage(adc_reading_Tamb, adc_chars);
        thermocouple_uV = ((esp_adc_cal_raw_to_voltage(adc_reading_Thermocouple_raw, adc_chars) * 1000) / GAIN_INA122P);
        ambient_temp = ((voltage_ambient - 500) / 10) - 6; //Value in degrees with respect to zero
        if (cnt == 4)
        {
            printf("Voltage: %d\tAmbient temperature: %d C\n", adc_reading_Tamb, ambient_temp);
            //printf("Raw: %d\tThermocuple voltage: %duV\n", adc_reading_Thermocouple_raw, thermocouple_uV);
            cnt = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xDelay250);
    }
}

void temperature_handle(void *pvParameters)
{
    /* Task that obtains temperatures from the thermocouple applying software compensation of the
    cold junction every 0.5seconds also an alarm is programmed to beep if the temperature surpass
    a set point and a led also indicates if the temperature is close to the setpoint. */
    const TickType_t xDelay500 = pdMS_TO_TICKS(500);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t a = 1;

    //Configure led, configure buzzer pins


    while (1)
    {
        if(adc_reading_Thermocouple_raw < 2)
        {   
            thermocouple_temp = V_to_ktemp(ambient_temp, 0);
        }

        else
        {
            thermocouple_temp = V_to_ktemp(ambient_temp, thermocouple_uV);
        }

        //TEST BUZZER
        if(enable == 1){

        if(a == 1)
        {
            gpio_set_level(Buzzer_GPIO, 1);
            a = 0;
        }

        else
        {
             gpio_set_level(Buzzer_GPIO, 0);
            a = 1;
        }
        }
        else{
        	gpio_set_level(Buzzer_GPIO, 0);
        }
        

       //printf("Thermocouple temperature: %d C\n",   thermocouple_temp);
       vTaskDelayUntil(&xLastWakeTime, xDelay500); 
    }
}

void led_fade(void *pvParameters)
{
	//LEDC
		/*
	     * Set configuration of timer0 for high speed channels
	     * that will be used by LED Controller
	     */

	ledc_timer_config_t ledc_timer = {
			.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
	        .freq_hz = 5000,                      // frequency of PWM signal
	        .speed_mode = LEDC_HS_MODE,           // timer mode
	        .timer_num =  LEDC_HS_TIMER,           // timer index
	        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
	    };

	ledc_channel_config_t ledc_channel= {
	        .channel    = LEDC_HS_CH0_CHANNEL,
	        .duty       = 0,
	        .gpio_num   = LED_GPIO,
	        .speed_mode = LEDC_HS_MODE,
	        .hpoint     = 0,
	        .timer_sel  = LEDC_HS_TIMER
	    };
	// Set LED Controller with previously prepared configuration
	ledc_timer_config(&ledc_timer);
	ledc_channel_config(&ledc_channel);
    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1) {
        ledc_set_fade_with_time(ledc_channel.speed_mode,
        ledc_channel.channel, LEDC_DUTY, LEDC_FADE_TIME);
        ledc_fade_start(ledc_channel.speed_mode,
        		ledc_channel.channel, LEDC_FADE_NO_WAIT);
        vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
        ledc_set_fade_with_time(ledc_channel.speed_mode,
        ledc_channel.channel, 0, LEDC_FADE_TIME);
        ledc_fade_start(ledc_channel.speed_mode,
        		ledc_channel.channel, LEDC_FADE_NO_WAIT);
        vTaskDelay(LEDC_FADE_TIME / portTICK_PERIOD_MS);
    }

}

void app_main(void)
{
    xTaskCreate(get_ADC_values, "get_ADC_values", 2000, NULL, 2, NULL); //If code crashes check stack depth
    xTaskCreate(temperature_handle, "temperature_handle", 2000, NULL, 1, NULL); //If code crashes check stack depth
    xTaskCreate(led_fade, "led_fade", 2000, NULL, 3, NULL);

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //Initialization for button with pullup and interrupt at rising edge

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO33
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(Button_GPIO, gpio_isr_handler, (void*) Button_GPIO);
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}
