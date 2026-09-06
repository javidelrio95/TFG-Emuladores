/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

//Librerias usadas
#include "core/lv_disp.h"
#include "driver/adc_types_legacy.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "hal/lv_hal_disp.h"
#include "hal/spi_flash_types.h"
#include "hal/uart_types.h"
#include "lvgl.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "soc/soc.h"
#include <ctype.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <esp_adc_cal.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "example";

//Definicion de los pines a los que estan conectados cada periferico
#define pulsadorOnda 4         
#define pulsadorReset 2
#define pulsadorEncendido 27
#define LED 26

//Definicion de los elementos que usaremos en el programa 
#define I2C_BUS_PORT 0
#define UART_NUM UART_NUM_2
#define UART_PC UART_NUM_0
#define BUFFER_SIZE 256
#define MEMORY 1024 * 2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your
///LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA 21
#define EXAMPLE_PIN_NUM_SCL 22
#define EXAMPLE_PIN_NUM_RST -1
#define EXAMPLE_I2C_HW_ADDR 0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES 128
#define EXAMPLE_LCD_V_RES 64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES 64
#define EXAMPLE_LCD_V_RES 128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

/*Definición de los canales del ADC1 para los valores de los potenciometros
 * configurados de manera externa*/
static adc_channel_t canales[3] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6};

/*Máximo valor digital que alcanzan los potenciometros*/
int maxPOT = 4095;

/*Definción de variables a usar en el proyecto*/
float valorFrecuencia = 0; // Valor de la frecuencia
float valorAmplitud = 0;   // Valor de la amplitud
float valorOffset = 0;     // Valor del offset
int tipo = 0;              // Forma de la señal a representar
bool reseteo = false;      // Variable para la funcion de reseteo 
// char *buffer;
bool ok = false;
int comando = 0;            
int output = 1;            // Valor de la salida
bool recibidoPC = false;   // Valor para saber si ha llegado un comando desde PC

//Valores actuales de los datos del generador
float FrAct = 0;           
float AmAct = 0;
float OfAct = 0;
int TiAct = 0;

//Valores donde se almacenan los datos leidos desde los perifericos de los emuladores
float FrecuHardware = 0;
float AmpHardware = 0;
float OfHardware = 0;
int TiHardware = 0;

//Estructura de datos del generador
typedef struct {
  float frecuencia;
  float amplitud;
  float offset;
  int forma;
} TDatos;

TDatos datos;
TDatos datosReset;
TDatos datosPantalla;

//LLamo al archivo donde se encuentra el codigo para representar por la pantalla OLED
extern void example_lvgl_demo_ui(lv_disp_t *disp, float frec, float ampli,
                                 float offset, float forma);

/*Subprogramas utilizados en el proyecto*/
esp_err_t configPulsadorOnda(void);
esp_err_t configPulsadorEncendido(void);
esp_err_t configLED(void);
esp_err_t configReset(void);
void init_uart(void);
void uart_pc(void);
lv_disp_t *oled_display();
void formaOnda(void *args);
void resetear(void *args);
void enviarDatos(void);
float Frecuencia(float ValorFrecuencia);
float Amplitud(float valorAmplitud);
float Offset(float valorOffset);

void app_main(void) {
  //Configuracion del ADC1	
  adc1_config_width(ADC_WIDTH_BIT_12);
  
  //Configuracion de los canales del ADC1
  for (int i = 0; i <= 2; ++i) {
    adc1_config_channel_atten(canales[i], ADC_ATTEN_DB_12);
  }

  int nivel;
  float acFr = 0, acAm = 0, acOf = 0, acFo = 0;

  configPulsadorEncendido();
  configLED();
  configPulsadorOnda();
  configReset();
  init_uart();
  uart_pc();
  lv_disp_t *disp = oled_display();
  uint8_t *datos_pc = (uint8_t *)malloc(1024 + 1);         //Reservo memoria
  int indice=0;                                            //Valor para leer comando recibido
  while (1) {                                              
    nivel = gpio_get_level(pulsadorEncendido);    //Compruebo el nivel del pulsador encendido    
    if (nivel == 1) {                                      //Si esta en modo activo 
      gpio_set_level(LED, 1);               //Enciendo led de encendido

      valorFrecuencia = adc1_get_raw(canales[2]);  //Leo desde el canal 2 del ADC1
      valorAmplitud = adc1_get_raw(canales[0]);    //Leo desde el canal 0 del ADC1
      valorOffset = adc1_get_raw(canales[1]);      //Leo desde el canal 1 del ADC1
      
      //Inicializo los valores de los datos de lectura de los perifericos
      float FrecuHardware = datos.frecuencia;
      float AmpHardware = datos.amplitud;
      float OfHardware = datos.offset;
      int TiHardware = datos.forma;
      
      //Filtro los valores de los datos del generador leidos por el ADC1
      //llamando a los subprogramas oportunos
      datos.forma = tipo;
      datos.frecuencia = Frecuencia(valorFrecuencia);
      datos.amplitud = Amplitud(valorAmplitud);
      datos.offset = Offset(valorOffset);
      
      //Si presiono el pulsador de reset
      if (reseteo == true) {
		//Almaceno en variables los valores acuales de los datos 
        acFr = datos.frecuencia;
        acAm = datos.amplitud;
        acOf = datos.offset;
        acFo = datos.forma;
        
        //Almaceno en una estructura los valores actuales de los datos 
        datosReset.frecuencia = acFr;
        datosReset.amplitud = acAm;
        datosReset.offset = acOf;
        datosReset.forma = acFo;

        //Muestro por pantalla los datos de reset
        datosPantalla.frecuencia = 1000;
        datosPantalla.amplitud = 0.1;
        datosPantalla.offset = 0;
        datosPantalla.forma = 0;

        reseteo = false;
      }

      //Declaracion de strings que se usaran para la lectura de comandos 
      char cmd[BUFFER_SIZE] = "";
      char cmd2[BUFFER_SIZE] = "";
      char idn[BUFFER_SIZE] = "";
      
      //Leo desde la UART0 
      int leer_pc = uart_read_bytes(UART_PC, datos_pc, BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
      
      //Si recibe desde la UART0
      if (leer_pc > 0) { 
		//Lo leo todo desde la UART0 hasta encontrar el caracter '\n' 
        for (int i = 0; i < leer_pc; ++i){               
          char caracter = (char)datos_pc[i];
          if(caracter == '\n'){
			  cmd[indice]='\0';
			  indice=0;
		  }else{
			  if(indice<BUFFER_SIZE-1){
				  cmd[indice]=caracter;
				  indice++;
			  }
		  }
        }
        //Paso el comando a minusculas
        for (unsigned i = 0; cmd[i] != '\0'; ++i) {
          cmd[i] = tolower(cmd[i]);
        }
        
        /*Proceso el comando de forma oportuna dependiendo de lo que haya llegado,
          idn muestra la identificacion del emulador y los datos se muestran con 3 
          decimales */
        recibidoPC = true;
        if (strcmp(cmd, "*idn?") == 0) {
          sprintf(idn, "GENERADOR EMULADO\r\n");
          uart_write_bytes(UART_PC, (const char *)idn, strlen(idn));
        } else if (strcmp(cmd, "source:frequency?") == 0) {
          float fr = datosPantalla.frecuencia;
          char afr[10];
          int len = sprintf(afr, "%.3f\r\n", fr);
          uart_write_bytes(UART_PC, (const char *)afr, len);
        } else if (strcmp(cmd, "source:amplitude?") == 0) {
          float a = datosPantalla.amplitud;
          char acar[8];
          int len = sprintf(acar, "%.3f\r\n", a);
          uart_write_bytes(UART_PC, (const char *)acar, len);
        } else if (strcmp(cmd, "source:offset?") == 0) {
          float o = datosPantalla.offset;
          char ocar[8];
          int len = sprintf(ocar, "%.3f\r\n", o);
          uart_write_bytes(UART_PC, (const char *)ocar, len);
        } else if (strcmp(cmd, "source:function?") == 0) {
          int f = datosPantalla.forma;
          if (f == 0) {
            char *formaOnda = "SINUSOID\r\n";
            uart_write_bytes(UART_PC, (const char *)formaOnda,
                             strlen(formaOnda));
          } else {
            char *formaOnda = "SQUARE\r\n";
            uart_write_bytes(UART_PC, (const char *)formaOnda,
                             strlen(formaOnda));
          }
        } else if (strcmp(cmd, "output?") == 0) {
          if (output == 1) {
            uart_write_bytes(UART_PC, "ON\r\n", strlen("ON\r\n"));
          } else {
            uart_write_bytes(UART_PC, "OFF\r\n", strlen("OFF\r\n"));
          }
          free(datos_pc);
        } else if (strcmp(cmd, "*rst") == 0) {
          datosPantalla.frecuencia = 1000;
          datosPantalla.amplitud = 0.1;
          datosPantalla.offset = 0;
          datosPantalla.forma = 0;

        } else {
		  /*Se lee el comando hasta que encuentra un espacio (tras el cual vendra el nuevo valor
		    del parametro a modificar) y el parametro se actualiza*/	
          int pos = 0;
          for (int i = 0; datos_pc[i] != ' '; ++i) {
            cmd2[i] = datos_pc[i];
            pos = i;
          }
          //Se pasa el comando a minuscula
          for (unsigned i = 0; i < strlen(cmd2); ++i) {
            cmd2[i] = tolower(cmd2[i]);
          }
          if (strcmp(cmd2, "source:frequency") == 0) {
            char valorf[10] = "";
            int j = 0;
            float valorfrec = 0;
            float frecuenciaActual = datos.frecuencia;
            for (int i = pos + 2; datos_pc[i] != '\n' && i<25; ++i) {
              valorf[j] = datos_pc[i];
              ++j;
              pos = j;
            }

            valorfrec = atof(valorf);
            // uart_write_bytes(UART_PC, "NUEVA FRECUENCIA\r\n", strlen("NUEVA
            // FRECUENCIA\r\n")); frecuenciaActual=datos.frecuencia;
            datosPantalla.frecuencia = valorfrec;

          } else if (strcmp(cmd2, "source:amplitude") == 0) {
            char valora[10] = "";
            int j = 0;
            float valoramp = 0;
            float amplitudActual = datos.amplitud;
            for (int i = pos + 2; datos_pc[i] != '\n' && i<25; ++i) {
              valora[j] = datos_pc[i];
              ++j;
              pos = j;
            }
            valoramp = atof(valora);
            // uart_write_bytes(UART_PC,"NUEVA AMPLITUD\r\n",strlen("NUEVA
            // AMPLITUD\r\n"));
            datosPantalla.amplitud = valoramp;
            
          } else if (strcmp(cmd2, "source:offset") == 0) {
            char valoro[10] = "";
            int j = 0;
            float valoroff = 0;
            float offsetActual = datos.offset;
            for (int i = pos + 2; datos_pc[i] != '\n' && i<25; ++i) {
              valoro[j] = datos_pc[i];
              ++j;
              pos = j;
            }
            valoroff = atof(valoro);
            // uart_write_bytes(UART_PC,"NUEVO OFFSET\r\n",strlen("NUEVO
            // OFFSET\r\n"));
            datosPantalla.offset = valoroff;
          } else if (strcmp(cmd2, "source:function") == 0) {
            char onda[10]="";
            int j = 0;
            int nivelOnda;
            // int ondaActual=datos.forma;
            for (int i = pos + 2; datos_pc[i] != '\n' && i<25; ++i) {
              onda[j] = datos_pc[i];
              ++j;
              pos = j;
            }
            // uart_write_bytes(UART_PC,(const char*)onda,strlen(onda));
            // uart_write_bytes(UART_PC,"\r\n", strlen("\r\n"));
            if (strcmp(onda, "SINUSOID") == 0) {
              nivelOnda = 0;
            } else {
              nivelOnda = 1;
            }
            datosPantalla.forma = nivelOnda;
          }else if (strcmp(cmd2, "output") == 0) {
            char ou[4];
            int j = 0;
            for (int i = pos + 2; datos_pc[i] != '\n'; ++i) {
              ou[j] = datos_pc[i];
              ++j;
              pos = j;
            }
            if (strcmp(ou, "on") == 0) {
              output = 1;
            }else {
              output = 0;
            }
          }

        }
        //vTaskDelay(100/portTICK_PERIOD_MS);
      }

      /*Mientras el valor del parametro no cambien sigue mostrando los valores reset,
        si se ha presionado el pulsador*/
      if ((datosReset.frecuencia == datos.frecuencia)&&(datosReset.amplitud == datos.amplitud)&&(datosReset.offset == datos.offset)/*&&(datosReset.forma == datos.forma)*/) {
           example_lvgl_demo_ui(disp, datosPantalla.frecuencia,datosPantalla.amplitud, datosPantalla.offset,datosPantalla.forma);
           vTaskDelay(100 / portTICK_PERIOD_MS);
      }else{
        /*Si se ha recibido un comando por PC, se muesran los valores de los parametros  
          de acuerdo al comando recibido*/ 
        if(recibidoPC == true){
          if((datos.frecuencia == FrecuHardware)&&(datos.amplitud == AmpHardware)&&(datos.offset == OfHardware)&&(datos.forma == TiHardware)){
               example_lvgl_demo_ui(disp, datosPantalla.frecuencia,datosPantalla.amplitud,datosPantalla.offset /*(valorFrecuencia&~(0x0000))*/,datosPantalla.forma);
               vTaskDelay(100 / portTICK_PERIOD_MS);
          }else{
               recibidoPC = false;
          }
        /*Si no se ha recibido comando, se comprueba si algun parametro a modificado su valor  
          y se muestra por pantalla o se mantiene igual si no se ha modificado*/
        }else{
		  if(FrecuHardware!=datos.frecuencia){
			  datosPantalla.frecuencia = datos.frecuencia;
		  }if(AmpHardware!=datos.amplitud){
	          datosPantalla.amplitud = datos.amplitud;
		  }if(OfHardware!=datos.offset){
	          datosPantalla.offset = datos.offset;
		  }if(TiHardware!=datos.forma){
	          datosPantalla.forma = datos.forma;  
		  }
          example_lvgl_demo_ui(disp,datosPantalla.frecuencia,datosPantalla.amplitud,datosPantalla.offset /*(valorFrecuencia&~(0x0000))*/,datosPantalla.forma);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
      }
      /*Si se ha modificado algun parametro se envia la info al osciloscopio por la UART2 
        */
      if((FrAct != datosPantalla.frecuencia)||(AmAct != datosPantalla.amplitud)||(OfAct != datosPantalla.offset)||(TiAct != datosPantalla.forma)){
        enviarDatos();
      }
      
      //Se actualiza el valor actual de los parametros
      FrAct = datosPantalla.frecuencia;
      AmAct = datosPantalla.amplitud;
      OfAct = datosPantalla.offset;
      TiAct = datosPantalla.forma;
      // recibidoPC=false;
    } else {                                   //Si esta en modo apagado
      gpio_set_level(LED, 0);   //Apago el led de encendido
      output = 0;      
    }

  }
  free(datos_pc);                             //Libero el espacio
}

/*Subprograma para configurar el puerto GPIO asignado al LED de encendido*/
esp_err_t configLED(void) {
  gpio_config_t led = {};
  led.pin_bit_mask = (1ULL << LED);
  led.mode = GPIO_MODE_OUTPUT;
  led.pull_up_en = GPIO_PULLUP_DISABLE;
  led.pull_down_en = GPIO_PULLDOWN_DISABLE;
  led.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&led);
  return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador que define la
 * foma de la onda*/
esp_err_t configPulsadorOnda(void) {
  gpio_config_t pulsador = {};
  pulsador.pin_bit_mask = (1ULL << pulsadorOnda);
  pulsador.mode = GPIO_MODE_INPUT;
  pulsador.pull_up_en = GPIO_PULLUP_ENABLE;
  pulsador.pull_down_en = GPIO_PULLDOWN_DISABLE;
  pulsador.intr_type = GPIO_INTR_NEGEDGE;
  gpio_config(&pulsador);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(pulsadorOnda, formaOnda, NULL);
  return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador de encendido*/
esp_err_t configPulsadorEncendido(void) {
  gpio_config_t encendido = {};
  encendido.pin_bit_mask = (1ULL << pulsadorEncendido);
  encendido.mode = GPIO_MODE_INPUT;
  encendido.pull_up_en = GPIO_PULLUP_DISABLE;
  encendido.pull_down_en = GPIO_PULLDOWN_DISABLE;
  encendido.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&encendido);
  return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador de reset*/
esp_err_t configReset(void) {
  gpio_config_t reset = {};
  reset.pin_bit_mask = (1ULL << pulsadorReset);
  reset.mode = GPIO_MODE_INPUT;
  reset.pull_up_en = GPIO_PULLUP_ENABLE;
  reset.pull_down_en = GPIO_PULLDOWN_DISABLE;
  reset.intr_type = GPIO_INTR_NEGEDGE;
  gpio_config(&reset);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(pulsadorReset, resetear, NULL);
  return ESP_OK;
}

/*Subprograma para configurar la UART2*/
void init_uart(void) {
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, 0);
}

/*Subprograma para configurar la UART0*/
void uart_pc(void) {
  uart_config_t uart_config_pc = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  uart_param_config(UART_PC, &uart_config_pc);
  uart_set_pin(UART_PC, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_PC, BUFFER_SIZE, BUFFER_SIZE, 0, NULL, 0);
}

/*Subprograma integrado en el espressif-IDE para la configuracion de la pantalla OLED*/
lv_disp_t *oled_display() {
  ESP_LOGI(TAG, "Initialize I2C bus");
  i2c_master_bus_handle_t i2c_bus = NULL;
  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .i2c_port = I2C_BUS_PORT,
      .sda_io_num = EXAMPLE_PIN_NUM_SDA,
      .scl_io_num = EXAMPLE_PIN_NUM_SCL,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_i2c_config_t io_config = {
    .dev_addr = EXAMPLE_I2C_HW_ADDR,
    .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    .control_phase_bytes = 1,               // According to SSD1306 datasheet
    .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
    .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    .dc_bit_offset = 6, // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    .dc_bit_offset = 0, // According to SH1107 datasheet
    .flags =
        {
            .disable_control_phase = 1,
        }
#endif
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

  ESP_LOGI(TAG, "Install SSD1306 panel driver");
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .bits_per_pixel = 1,
      .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
  };
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif
  ESP_LOGI(TAG, "Initialize LVGL");
  const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  lvgl_port_init(&lvgl_cfg);

  const lvgl_port_display_cfg_t disp_cfg = {.io_handle = io_handle,
                                            .panel_handle = panel_handle,
                                            .buffer_size = EXAMPLE_LCD_H_RES *
                                                           EXAMPLE_LCD_V_RES,
                                            .double_buffer = true,
                                            .hres = EXAMPLE_LCD_H_RES,
                                            .vres = EXAMPLE_LCD_V_RES,
                                            .monochrome = true,
                                            .rotation = {
                                                .swap_xy = false,
                                                .mirror_x = false,
                                                .mirror_y = false,
                                            }};
  lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);

  /* Rotation of the screen */
  lv_disp_set_rotation(disp, LV_DISP_ROT_180);

  ESP_LOGI(TAG, "Display LVGL Scroll Text");
  return disp;
}

/*Subprograma para cambiar el valor del pulsador de forma de onda*/
void formaOnda(void *args){
	 tipo = 1 - tipo; 	 
}

/*Subprograma para enviar datos por la UART2*/
void enviarDatos(void) {
  ok = true;
  TDatos da;
  da.frecuencia = datosPantalla.frecuencia;
  da.amplitud = datosPantalla.amplitud;
  da.offset = datosPantalla.offset;
  da.forma = datosPantalla.forma;

  char *car1 = "F";
  char *car2 = "A";
  char *car3 = "O";
  char *car4 = "R";
  
  //Los datos se enviaran con 3 decimales
  char da1[BUFFER_SIZE];
  sprintf(da1, "%.3f", da.frecuencia);

  char da2[BUFFER_SIZE];
  sprintf(da2, "%.3f", da.amplitud);

  char da3[BUFFER_SIZE];
  sprintf(da3, "%.3f", da.offset);

  char da4[BUFFER_SIZE];
  itoa(da.forma, da4, 10);
  
  /*Envio el caracer F, luego el valor de la frecuencia, posteriormente la A
   y luego su valor, y asi igual con la O y la R, que son el offset y la
   forma de onda*/
  uart_write_bytes(UART_NUM_2, (const char *)car1, strlen(car1));
  uart_write_bytes(UART_NUM_2, (const char *)da1, strlen(da1));
  uart_write_bytes(UART_NUM_2, (const char *)car2, strlen(car2));
  uart_write_bytes(UART_NUM_2, (const char *)da2, strlen(da2));
  uart_write_bytes(UART_NUM_2, (const char *)car3, strlen(car3));
  uart_write_bytes(UART_NUM_2, (const char *)da3, strlen(da3));
  uart_write_bytes(UART_NUM_2, (const char *)car4, strlen(car4));
  uart_write_bytes(UART_NUM_2, (const char *)da4, strlen(da4));
}

/*Subprograma para cambiar el valor del parametro de reset*/
void resetear(void *args){
	 reseteo = true;
}

/*Subprograma usado para el escalamiento de los valores del potenciometro
 * correspondiente a la frecuencia*/
float Frecuencia(float valorFrecuencia) {

  float frec = 0; // Declaro una variable nueva llamada frec y la inicializo

  int minSal = 0; // Valor minimo que puede alcanzar nuestra frecuencia una vez
                  // hecho el escalamiento
  int maxSal = 20; // Valor maximo que puede alcanzar nuestra frecuencia una vez
                   // hecho el escalamiento

  /*Realizamos el escalamiento usando la siguiente fórmula matematica y teniendo
    en cuanta los valores minimos y maximos que alcanzará nuestra frecuencia una
    vez escalada y el valor máximo que puede alcanzar nuestro potenciometro
    digitalmente*/
  frec = ((valorFrecuencia * (maxSal - minSal)) / (maxPOT) + minSal) * 500;
 
  //Hacemos un filtrado final para un paso de 500 Hz dependiendo del valor final mapeado
  //con esto evitamos que los numeros fluctuen al ser muchos valores
  if (frec == 0) {
    frec = 1;
  } else if (frec > 0 && frec <= 500) {
    frec = 500;
  } else if (frec > 500 && frec <= 1000) {
    frec = 1000;
  } else if (frec > 1000 && frec <= 1500) {
    frec = 1500;
  } else if (frec > 1500 && frec <= 2000) {
    frec = 2000;
  } else if (frec > 2000 && frec <= 2500) {
    frec = 2500;
  } else if (frec > 2500 && frec <= 3000) {
    frec = 3000;
  } else if (frec > 3000 && frec <= 3500) {
    frec = 3500;
  } else if (frec > 3500 && frec <= 4000) {
    frec = 4000;
  } else if (frec > 4000 && frec <= 4500) {
    frec = 4500;
  } else if (frec > 4500 && frec <= 5000) {
    frec = 5000;
  } else if (frec > 5000 && frec <= 5500) {
    frec = 5500;
  } else if (frec > 5500 && frec <= 6000) {
    frec = 6000;
  } else if (frec > 6000 && frec <= 6500) {
    frec = 6500;
  } else if (frec > 6500 && frec <= 7000) {
    frec = 7000;
  } else if (frec > 7000 && frec <= 7500) {
    frec = 7500;
  } else if (frec > 7500 && frec <= 8000) {
    frec = 8000;
  } else if (frec > 8000 && frec <= 8500) {
    frec = 8500;
  } else if (frec > 8500 && frec <= 9000) {
    frec = 9000;
  } else if (frec > 9000 && frec <= 9500) {
    frec = 9500;
  } else if (frec > 9500 && frec <= 10000) {
    frec = 10000;
  }
  return frec; // Devolvemos el valor de la frecuencia escalada
}

/*Subprograma usado para el escalamiento de los valores del potenciometro
 * correspondiente a la amplitud*/
float Amplitud(float valorAmplitud) {

  float amp = 0; // Decalro una variables nueva llamada amp y la inicializo

  int minSal = 0; // Valor minimo que puede alcanzar nuestra amplitud una vez
                  // hecho el escalamiento
  int maxSal = 10; // Valor máximo que puede alcanzar nuestra amplitud una vez
                   // hecho el escalamiento

  /*Realizamos el escalamiento usando la fórmula matemática que configuramos
   * previamente para escalar la frecuencia*/
  amp = (valorAmplitud * (maxSal - minSal)) / (maxPOT) + minSal;

  //Hacemos un filtrado final para que no fluctuen los valores
  if (amp == 0) {
    amp = 0.1;
  } else if (amp > 0 && amp <= 1) {
    amp = 1;
  } else if (amp > 1 && amp <= 2) {
    amp = 2;
  } else if (amp > 2 && amp <= 3) {
    amp = 3;
  } else if (amp > 3 && amp <= 4) {
    amp = 4;
  } else if (amp > 4 && amp <= 5) {
    amp = 5;
  } else if (amp > 5 && amp <= 6) {
    amp = 6;
  } else if (amp > 6 && amp <= 7) {
    amp = 7;
  } else if (amp > 7 && amp <= 8) {
    amp = 8;
  } else if (amp > 8 && amp <= 9) {
    amp = 9;
  } else if (amp > 9 && amp <= 10) {
    amp = 10;
  }

  return amp; // Devolvemos el valor de la amplitud escalada
}

/*Subprograma usado para el escalamiento de los valores del potenciometro
 * correspondiente al offset*/
float Offset(float valorOffset) {

  float off = 0; // Declaro una variable nueva llamada off y la inicializo

  int minSal = -5; // Valor minimo que puede alcanzar nuestro offset una vez
                   // hecho el escalamiento
  int maxSal = 5; // Valor máximo que puede alcanzar nuestro offset una vez
                  // hecho el escalamiento

  /*Realizamos el escalamiento usando la fórmula matemática que configuramos
   * previamente para escalar la frecuencia*/
  off = (valorOffset * (maxSal - minSal)) / (maxPOT) + minSal;

  //Hacemos un filtrado final para que no fluctuen los valores
  if (off == -5) {
    off = -5;
  } else if (off > -5 && off <= -4) {
    off = -4;
  } else if (off > -4 && off <= -3) {
    off = -3;
  } else if (off > -3 && off <= -2) {
    off = -2;
  } else if (off > -2 && off <= -1) {
    off = -1;
  } else if (off > -1 && off <= 0) {
    off = 0;
  } else if (off > 0 && off <= 1) {
    off = 1;
  } else if (off > 1 && off <= 2) {
    off = 2;
  } else if (off > 2 && off <= 3) {
    off = 3;
  } else if (off > 3 && off <= 4) {
    off = 4;
  } else if (off > 4 && off <= 5) {
    off = 5;
  }

  return off; // DEvolvemos el valor del offset escalado
}
