/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

//Librerias usadas
#include <stdint.h>
#include <stdio.h>
#include "core/lv_disp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "hal/lv_hal_disp.h"
#include "hal/uart_types.h"
#include "lvgl.h"
#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_adc_cal.h>
#include <driver/i2c.h>
#include "driver/adc_types_legacy.h"
#include "esp_err.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "rom/uart.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <driver/uart.h>
#include <string.h>
#include <sys/_intsup.h>
#include <ctype.h>

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "example";

//Definicion de los pines a los que estan conectados cada periferico
#define pulsadorEscala 4
#define pulsadorReset 2	
#define pulsadorEncendido 27	
#define LED 26	

//Definicion de los elementos que usaremos en el programa
#define I2C_BUS_PORT  0
#define UART_NUM UART_NUM_2
#define UART_PC UART_NUM_0
#define BUFFER_SIZE 256
#define MEMORY 1024 * 2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

/*Definición de los canales del ADC1 para los valores de los potenciometros configurados de manera externa*/
static adc_channel_t canales[3] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6};

/*Máximo valor digital que alcanzan los potenciometros*/
int maxPOT = 4095;

/*Definción de variables a usar en el proyecto*/
float valorV = 0;		//Valor de la escala vertical
float valorMS = 0;		//Valor de la escala horizontal
float valorDiv = 0;		//Valor de la posicion vertical
int valorEscala = 0;	//Valor de la escala
bool reseteo = false;
//char *buffer;
bool recibido = false;  //Valor para saber si se a recibido por la UART2
bool recibidoPC=false;  //Valor para saber si se a recibido por la UART0
//int tipo=0;
bool tipo=false;

//Valores donde se almacenan los datos leidos desde los perifericos de los emuladores
float VolHardware=0;
float MSHardware=0;
float DivHardware=0;
int EscHardware=0;

//Estructura de datos del osciloscopio
typedef struct{
	float voltios;
	float ms;
	float div;
	int escala;
}TDatos;

TDatos datos;
TDatos datosReset;
TDatos datosPantalla;

//Estructura de datos que llega desde el generador
typedef struct{
	float frecuencia;
	float amplitud;
	float offset;
	int forma;
}TDatosGen;

TDatosGen datos_gen;

//LLamo al archivo donde se encuentra el codigo para representar por la pantalla OLED
extern void example_lvgl_demo_ui(lv_disp_t *disp, float voltios, float ms, float div, int escala, bool recibido);

/*Subprogramas utilizados en el proyecto*/
esp_err_t configPulsadorEscala(void);
esp_err_t configPulsadorEncendido(void);
esp_err_t configLED(void);
esp_err_t configReset(void);
esp_err_t configPulsadorUART(void);
void init_uart(void);
lv_disp_t *oled_display();
void EscalaOnda(void *args);
void uart_pc(void);
void resetear(void *args);
void enviarDatos(void *args);
float Volts(float valorV);
float MS(float valorMS);
float Division(float valorDiv);

void app_main(void)
{
	//Configuracion del ADC1
	adc1_config_width(ADC_WIDTH_BIT_12);
	
	//Configuracion de los canales del ADC1
	for(int i=0;i<=2;++i){
		adc1_config_channel_atten(canales[i], ADC_ATTEN_DB_12);
	}
	
	int nivel;
	float acV=0, acMS=0,acDiv=0;
	int acEscala=0;


	configPulsadorEncendido();
	configLED();
	configPulsadorEscala();
	configReset();
	init_uart();
	uart_pc();
	lv_disp_t *disp = oled_display();
	uint8_t* data = (uint8_t*) malloc(1024 + 1);	     //Reservo memoria para los datos del generador
	uint8_t* datos_pc_osci = (uint8_t*)malloc(1024+1);   //Reservo memoria para los comandos que llegan por pc
	int indice=0;
	while(1){
		nivel = gpio_get_level(pulsadorEncendido);  //Compruebo el nivel del pulsador encendido
		if(nivel == 1){                                      //Si esta en modo activo 
			gpio_set_level(LED,1);            //Enciendo led de encendido
			valorV = adc1_get_raw(canales[2]);       //Leo desde el canal 2 del ADC1  
			valorMS = adc1_get_raw(canales[0]);      //Leo desde el canal 0 del ADC1
			valorDiv = adc1_get_raw(canales[1]);     //Leo desde el canal 1 del ADC1
			
			//Inicializo los valores de los datos de lectura de los perifericos
			float VolHardware=datos.voltios;
            float MSHardware=datos.ms; 
            float DivHardware=datos.div;
            int EscHardware=datos.escala;
			
			
			//Filtro los valores de los datos del osciloscopio leidos por el ADC1
            //llamando a los subprogramas oportunos
			datos.escala = valorEscala;					
			datos.voltios = Volts(valorV);
			datos.ms = MS(valorMS);
			datos.div = Division(valorDiv);
			
			//Si presiono el pulsador de reset
			if(reseteo==true){
				//Almaceno en variables los valores acuales de los datos
				acV=datos.voltios;
				acMS=datos.ms;
				acDiv=datos.div;
				acEscala=datos.escala;
				
				//Almaceno en una estructura los valores actuales de los datos 
				datosReset.voltios=acV;
				datosReset.ms=acMS;
				datosReset.div=acDiv;
				datosReset.escala=acEscala;
				
				//Muestro por pantalla los datos de reset
				datosPantalla.voltios=0.1;
				datosPantalla.ms=0.1;
				datosPantalla.div=0;
				datosPantalla.escala=1;
				
				reseteo=false;
				
		   }else{
			   /*Aqui leermeos desde la UART2 todos los datos que nos envie
			     el generador para asi poder mostrarlos por la pantalla OLED*/
			   int i=0, j=0, k=0;
			   int act=0;
			   //Declaro strings para cada uno de los parametros
			   char car1[20]="";
			   char car2[10]="", car3[10]="", car4=' ';		
			   //Leo desde la UART2   
			   const int leer = uart_read_bytes(UART_NUM_2, data, BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
			   if(leer > 0){				   
					   //Leo todo el primer parameros hasta encontrar el caracter de separacion
					   //y lo almaceno en car1
					   for(i=1;data[i]!='A';++i){
						   car1[i-1]=data[i];
						   act=i;
					   }
					   //Leo todo el primer parameros hasta encontrar el caracter de separacion
					   //y lo almaceno en car2					   		   
					   for(i=act+2;data[i]!='O';++i){				   
						   car2[j]=data[i];
						   act=i;
						   j++;
					   }
					   //Leo todo el primer parameros hasta encontrar el caracter de separacion
					   //y lo almaceno en car3					   
					   for(i=act+2;data[i]!='R';++i){
						   car3[k]=data[i];
						   act=i;
						   k++;
					   }
					   //Leo todo el primer parameros hasta encontrar el caracter de separacion
					   //y lo almaceno en car4					   
					   for(i=act+2;i<leer;++i){
						   car4=data[i];
					   }
					   //recibido=true;
					   
					   //Asigno a la estructura de los datos del generador
					   //los valores recibidos
					   datos_gen.frecuencia=atof(car1);
					   datos_gen.amplitud=atof(car2);
					   datos_gen.offset=atof(car3);
					   datos_gen.forma=car4 - '0';
					   //vTaskDelay(100/portTICK_PERIOD_MS);
						   
					   
					 
					   
					   /*example_lvgl_demo_ui(disp, datos_gen.frecuencia, datos_gen.amplitud, datos_gen.offset, datos_gen.forma,recibido);
				   	   vTaskDelay(100/portTICK_PERIOD_MS);
				   	   
				   	   
				   	   recibido=false;*/	   	   
			 }
			 
			 //Declaracion de strings que se usaran para la lectura de comandos
			 char cmd[BUFFER_SIZE]="";
			 char idn[BUFFER_SIZE]="";
			 char cmd2[BUFFER_SIZE]="";
			 //Leo desde la UART0
			 const int leer_pc=uart_read_bytes(UART_PC, datos_pc_osci,BUFFER_SIZE, 100 / portTICK_PERIOD_MS);
			 //Si recibe desde la UART0
			 if(leer_pc>0){
				 //Lo leo todo desde la UART0 hasta encontrar el caracter '\n' 
				 for (int i = 0; i < leer_pc; ++i){
			        char caracter = (char)datos_pc_osci[i];
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
                   recibidoPC=true;
				   if(strcmp(cmd,"*idn?")==0){
					   sprintf(idn,"OSICLOSCOPIO EMULADO\r\n");
					   uart_write_bytes(UART_PC,(const char*)idn, strlen(idn));
				   }else if(strcmp(cmd,"ch1:probe?")==0){
					   int e=datos.escala;
					   if(e==0){
						   char* escalaOsci="1\r\n";
						   uart_write_bytes(UART_PC, (const char*)escalaOsci, strlen(escalaOsci));
					   }else{
						   char* escalaOsci="10\r\n";
						   uart_write_bytes(UART_PC, (const char*)escalaOsci, strlen(escalaOsci)); 
					   }
				   }else if(strcmp(cmd,"ch1:scale?")==0){
					   float v=datosPantalla.voltios;
					   char vcar[8];
					   int len=sprintf(vcar, "%.3f\r\n", v);
					   uart_write_bytes(UART_PC, (const char*)vcar, len);
				   }else if(strcmp(cmd,"ch1:position?")==0){
					   float p=datosPantalla.div;
					   char pcar[8];
					   int len=sprintf(pcar,"%.3f\r\n",p);
					   uart_write_bytes(UART_PC, (const char*)pcar, len);
				   }else if(strcmp(cmd,"horizontal:main:scale?")==0){
					   float mili=datosPantalla.ms;
					   char milicar[8];
					   int len=sprintf(milicar,"%.3f\r\n",mili);
					   uart_write_bytes(UART_PC, (const char*)milicar, len);
				   }else if(strcmp(cmd,"measurement:immed:type?")==0){ 
					   if(tipo==false){
						   uart_write_bytes(UART_PC, "FREQUENCY\r\n", strlen("FREQUENCY\r\n"));
					   }else{
						   uart_write_bytes(UART_PC, "PK2PK\r\n", strlen("PK2PK\r\n"));
					   }
				   }else if(strcmp(cmd,"measurement:immed:value?")==0){
					   if(tipo==false){
						   char fcar[10];
						   float f=datos_gen.frecuencia;
						   int len=sprintf(fcar,"%.3f\r\n",f);
						   uart_write_bytes(UART_PC, (const char*)fcar, len);
					   }else{
						   char acar[8];	
						   float a=2*datos_gen.amplitud;
						   int len=sprintf(acar, "%.3f\r\n",a);
						   uart_write_bytes(UART_PC, (const char*)acar, len);
					   }
				   }else if(strcmp(cmd,"*rst")==0){
					   datosPantalla.voltios=0.1;
					   datosPantalla.ms=0.1;
					   datosPantalla.div=0;
					   datosPantalla.escala=1; 
					   				   	   
				   }
				   else{
		               /*Se lee el comando hasta que encuentra un espacio (tras el cual vendra el nuevo valor
		               del parametro a modificar) y el parametro se actualiza*/						   
					   int pos=0;
					   for(int i=0;datos_pc_osci[i]!=' ';++i){
						   cmd2[i]=datos_pc_osci[i];
						   pos=i;
					   }
					   //Se pasa el comando a minuscula
					   for (unsigned i = 0; i < strlen(cmd2); ++i) {
                           cmd2[i] = tolower(cmd2[i]);
                       }
					   if(strcmp(cmd2,"ch1:scale")==0){
						   char valorv[8]="";
						   int j=0;
						   float valorVol=0;
						   float voltiosActual=datos.voltios;
						   for(int i=pos+2;datos_pc_osci[i]!='\n';++i){
							   valorv[j]=datos_pc_osci[i];
							   ++j;
							   pos=j;
						   }
						   
						   valorVol=atof(valorv); 
						   datosPantalla.voltios=valorVol;
					   }else if(strcmp(cmd2,"ch1:position")==0){
						   char valordiv[4]="";
						   int j=0;
						   float valorD=0;
						   float divActual=datos.div;
						   for(int i=pos+2;datos_pc_osci[i]!='\n';++i){
							   valordiv[j]=datos_pc_osci[i];
							   ++j;
							   pos=j;
						   }
						   
						   valorD=atof(valordiv); 
						   datosPantalla.ms=valorD;
					   }else if(strcmp(cmd2,"horizontal:main:scale")==0){
						   char valormili[4]="";
						   int j=0;
						   float valorMili=0;
						   float miliActual=datos.ms;
						   for(int i=pos+2;datos_pc_osci[i]!='\n';++i){
							   valormili[j]=datos_pc_osci[i];
							   ++j;
							   pos=j;
						   }
						   
						   valorMili=atof(valormili); 
						   datosPantalla.ms=valorMili;   
					   }else if(strcmp(cmd2,"measurement:immed:type")==0){
						   char ti[20]="";
						   int j=0;
						   for(int i=pos+2;datos_pc_osci[i]!='\n';++i){
							   ti[j]=datos_pc_osci[i];
							   ++j;
							   pos=j;
						   }
						   for(int i=0;i<strlen(ti);++i){
							   ti[i]=tolower(ti[i]);
						   }
						   if(strcmp(ti,"frequency")==0){
							   tipo=false;
						   }else{							   
							   tipo=true;
						   }
					   }
				   }
				   //vTaskDelay(100/portTICK_PERIOD_MS);
			 }
             /*Mientras el valor del parametro no cambien sigue mostrando los valores reset,
              si se ha presionado el pulsador*/			 
			 if((datosReset.voltios==datos.voltios)&&(datosReset.ms==datos.ms)&&(datosReset.div==datos.div)&&(datosReset.escala==datos.escala)){
			   example_lvgl_demo_ui(disp,datosPantalla.voltios,datosPantalla.ms,datosPantalla.div,datosPantalla.escala, recibido);
		       vTaskDelay(100/portTICK_PERIOD_MS);
			 }else if(datos.div==1){
				 //Si esa en modo generador que es estando el tercer potenciometro en valor 1
				 //se representa lo que llegó por la UART2 o 0 si no ha llegado nada previamente
				 recibido=true;
				 example_lvgl_demo_ui(disp,datos_gen.frecuencia,datos_gen.amplitud,datos_gen.offset /*(valorFrecuencia&~(0x0000))*/,datos_gen.forma,recibido);
				 vTaskDelay(100/portTICK_PERIOD_MS);
			 }else{	
				 //No ha llegado nada por la UART2			 
				 recibido=false;
				 /*Si se ha recibido un comando por PC, se muesran los valores de los parametros  
                  de acuerdo al comando recibido*/ 
				 if(recibidoPC==true){
					if((datos.voltios==VolHardware)&&(datos.ms==MSHardware)&&(datos.div==DivHardware)&&(datos.escala==EscHardware)){					
						example_lvgl_demo_ui(disp,datosPantalla.voltios,datosPantalla.ms,datosPantalla.div/*(valorFrecuencia&~(0x0000))*/,datosPantalla.escala,recibido);
				        vTaskDelay(100/portTICK_PERIOD_MS);
				    }else{
                        recibidoPC=false;
                    }
     
			        
				}else{
					 /*Si no se ha recibido comando, se comprueba si algun parametro a modificado su valor  
                      y se muestra por pantalla o se mantiene igual si no se ha modificado*/
					  if(VolHardware!=datos.voltios){
						  datosPantalla.voltios = datos.voltios;
					  }if(MSHardware!=datos.ms){
				          datosPantalla.ms = datos.ms;
					  }if(DivHardware!=datos.div){
				          datosPantalla.div = datos.div;
					  }if(EscHardware!=datos.escala){
				          datosPantalla.escala = datos.escala;  
					  }
			          example_lvgl_demo_ui(disp,datosPantalla.voltios,datosPantalla.ms,datosPantalla.div /*(valorFrecuencia&~(0x0000))*/,datosPantalla.escala,recibido);
			          vTaskDelay(100 / portTICK_PERIOD_MS);
               }
			   
			 }
 
	         
		 }
		   
		   // Lock the mutex due to the LVGL APIs are not thread-safe

		   
		}else{    //Si esta en modo apagado                      
			gpio_set_level(LED,0); //Apago el led
		}
  	}
  	//Libero el espacio
  	free(data);
  	free(datos_pc_osci);
}

/*Subprograma para configurar el puerto GPIO asignado al LED de encendido*/
esp_err_t configLED(void){
	gpio_config_t led = {};							
	led.pin_bit_mask = (1ULL<<LED);
	led.mode = GPIO_MODE_OUTPUT;
	led.pull_up_en = GPIO_PULLUP_DISABLE;
	led.pull_down_en = GPIO_PULLDOWN_DISABLE;
	led.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&led);
	return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador que define la foma de la onda*/
esp_err_t configPulsadorEscala(void){
	gpio_config_t pulsador = {};
	pulsador.pin_bit_mask = (1ULL<<pulsadorEscala);
	pulsador.mode = GPIO_MODE_INPUT;
	pulsador.pull_up_en = GPIO_PULLUP_ENABLE;
	pulsador.pull_down_en = GPIO_PULLDOWN_DISABLE;
	pulsador.intr_type = GPIO_INTR_NEGEDGE;
	gpio_config(&pulsador);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(pulsadorEscala, EscalaOnda, NULL);
	return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador de encendido*/
esp_err_t configPulsadorEncendido(void){
	gpio_config_t encendido = {};
	encendido.pin_bit_mask = (1ULL<<pulsadorEncendido);
	encendido.mode = GPIO_MODE_INPUT;
	encendido.pull_up_en = GPIO_PULLUP_DISABLE;
	encendido.pull_down_en = GPIO_PULLDOWN_DISABLE;
	encendido.intr_type = GPIO_INTR_DISABLE;
	gpio_config(&encendido);
	return ESP_OK;
}

/*Subprograma para configurar el puerto GPIO asignado al pulsador de reset*/
esp_err_t configReset(void){
	gpio_config_t reset = {};
	reset.pin_bit_mask = (1ULL<<pulsadorReset);
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
void init_uart(void){
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	
	uart_param_config(UART_NUM_2,&uart_config);
	uart_set_pin(UART_NUM_2, 17,16,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_2,BUFFER_SIZE,BUFFER_SIZE,0,NULL,0);
}

/*Subprograma para configurar la UART0*/
void uart_pc(void){
	uart_config_t uart_config_pc = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	
	uart_param_config(UART_PC,&uart_config_pc);
	uart_set_pin(UART_PC, 1,3,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_PC,BUFFER_SIZE,BUFFER_SIZE,0,NULL,0);	
}

/*Subprograma integrado en el espressif-IDE para la configuracion de la pantalla OLED*/
lv_disp_t *oled_display(){
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
		        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
		#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
		        .dc_bit_offset = 0,                     // According to SH1107 datasheet
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
		    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
		#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
		    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
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
	
	    const lvgl_port_display_cfg_t disp_cfg = {
	        .io_handle = io_handle,
	        .panel_handle = panel_handle,
	        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
	        .double_buffer = true,
	        .hres = EXAMPLE_LCD_H_RES,
	        .vres = EXAMPLE_LCD_V_RES,
	        .monochrome = true,
	        .rotation = {
	            .swap_xy = false,
	            .mirror_x = false,
	            .mirror_y = false,
	        }
	    };
	    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
	
	    /* Rotation of the screen */
	    lv_disp_set_rotation(disp, LV_DISP_ROT_180);
	
	    ESP_LOGI(TAG, "Display LVGL Scroll Text");
	    return disp;	
}

/*Subprograma para cambiar el valor de la escala*/
void EscalaOnda(void *args){
	valorEscala = 1 - valorEscala;	
}

/*Subprograma para cambiar el valor del parametro de reset*/
void resetear(void *args){	
	reseteo=true;
}

/*Subprograma usado para el escalamiento de los valores del potenciometro correspondiente a la frecuencia*/
float Volts(float valorV){
	
	float vol = 0;      		//Declaro una variable nueva llamada frec y la inicializo
	
	int minSal = 0;			//Valor minimo que puede alcanzar nuestra frecuencia una vez hecho el escalamiento
	int maxSal = 6;		//Valor maximo que puede alcanzar nuestra frecuencia una vez hecho el escalamiento
	
	/*Realizamos el escalamiento usando la siguiente fórmula matematica y teniendo en cuanta los valores 
	  minimos y maximos que alcanzará nuestra frecuencia una vez escalada y el valor máximo que puede alcanzar
	  nuestro potenciometro digitalmente*/
	vol = (valorV *(maxSal - minSal))/(maxPOT) + minSal;
	
	//Hacemos un filtrado final para los saltos no lineales dependiendo del valor y que
    //con esto los valores a mostrar sean los que queremos
	if(vol==0){
		vol=0.1;
	}else if(vol>0 && vol<=1){
		vol=0.2;
	}else if(vol>1 && vol<=2){
		vol=0.5;
	}else if(vol>2 && vol<=3){
		vol=1;
	}else if(vol>3 && vol<=4){
		vol=2;
	}else if(vol>4 && vol<=5){
		vol=5;
	}else if(vol>5 && vol<=6){
		vol=10;
	}
	
	return vol;			//Devolvemos el valor de la frecuencia escalada
}

/*Subprograma usado para el escalamiento de los valores del potenciometro correspondiente a la amplitud*/
float MS(float valorMS){
	
	float milisec = 0;			//Decalro una variables nueva llamada amp y la inicializo
	
	int minSal = 0;			//Valor minimo que puede alcanzar nuestra amplitud una vez hecho el escalamiento
	int maxSal = 9;		//Valor máximo que puede alcanzar nuestra amplitud una vez hecho el escalamiento
	
	/*Realizamos el escalamiento usando la fórmula matemática que configuramos previamente para escalar la frecuencia*/
	milisec = ((valorMS * (maxSal - minSal))/(maxPOT) + minSal);
	
	//Hacemos un filtrado final para los saltos no lineales dependiendo del valor y que
    //con esto los valores a mostrar sean los que queremos
	if(milisec==0){
		milisec=0.1;
	}else if(milisec>0 && milisec<=1){
		milisec=0.25;
	}else if(milisec>1 && milisec<=2){
		milisec=0.5;
	}else if(milisec>2 && milisec<=3){
		milisec=1;
	}else if(milisec>3 && milisec<=4){
		milisec=2.5;
	}else if(milisec>4 && milisec<=5){
		milisec=5;
	}else if(milisec>5 && milisec<=6){
		milisec=10;
	}else if(milisec>6 && milisec<=7){
		milisec=25;
	}else if(milisec>7 && milisec<=8){
		milisec=50;
	}else if(milisec>8 && milisec<=9){
		milisec=100;
	}
	
	return milisec;				//Devolvemos el valor de la amplitud escalada
}

/*Subprograma usado para el escalamiento de los valores del potenciometro correspondiente al offset*/
float Division(float valorDiv){
	
	float divi = 0;			//Declaro una variable nueva llamada off y la inicializo

	int minSal = -5;		//Valor minimo que puede alcanzar nuestro offset una vez hecho el escalamiento
	int maxSal = 5;			//Valor máximo que puede alcanzar nuestro offset una vez hecho el escalamiento
	
	/*Realizamos el escalamiento usando la fórmula matemática que configuramos previamente para escalar la frecuencia*/
	divi = (valorDiv * (maxSal - minSal))/(maxPOT) + minSal;
	
	//Con esto realizamos un filrado para que solo haya dos valores posibles
	if(divi<=0){
		divi=0;
	}else if(divi>0){
		divi=1;
	}
	
	return divi;				//DEvolvemos el valor del offset escalado
	
}
