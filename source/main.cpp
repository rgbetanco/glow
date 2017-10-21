/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "LEDService.h"
#include "UARTService.h"
#include "fds.h"
#include "app_pwm.h"

#define NEED_CONSOLE_OUTPUT 0 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */
#define COMBO_EFFECT 5
#define PULSE_EFFECT 3
#if NEED_CONSOLE_OUTPUT
#define DEBUG(STR) { if (uart) uart->write(STR, strlen(STR)); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define NUM_PAGES 4

//DigitalIn PWR_DET(BUTTON3);         		//19
DigitalIn PWR_DET(p19);         			//19
//InterruptIn BT_PWR_DET(BUTTON3);    		//19
InterruptIn BT_PWR_DET(p19);    			//19

//DigitalOut SYS_CE(p28);          			//26 - P0.26 and P0.27 are by default used for the 32 kHz crystal and are not available on the connectors
DigitalOut SYS_CE(p26);          			//26 - P0.26 and P0.27 are by default used for the 32 kHz crystal and are not available on the connectors
//DigitalOut PWM_CH1(LED1);            		//12
DigitalOut PWM_CH1(p12);                    //12  
//DigitalOut PWM_CH2(LED2);            		//13
DigitalOut PWM_CH2(p13);					//13  
//PwmOut CTRL_A(LED3);             			//18
PwmOut CTRL_A(p18);             			//18
//PwmOut CTRL_B(LED4);             			//17
PwmOut CTRL_B(p17);             			//17

Serial pc(USBTX, USBRX);

BLEDevice  ble;
UARTService *uart;

const static char     DEVICE_NAME[] = "GLOWB";
static const uint16_t uuid16_list[] = {LEDService::LED_SERVICE_UUID};

uint8_t counter = 0;
uint8_t setPeriod = 0;
uint8_t setEffect = 0;
uint8_t Abrightness = 100;
uint8_t Bbrightness = 100;
double flashon = 0.1;
double flashoff = 0.8;
double steadyondelay = 0.001;                                                   //delay for steady on function
double twinkledelayone = 0.1;
double twinkledelayfour = 0.4;
double flashdelay = 0.5;
double chBrightness = 0.6;

//temp
uint8_t lbright = 100;
uint8_t leffect = 0;
uint8_t seffect = 0;

short defaultHours = 4;
short scheduleFlag = 0;
short scheduleEffect = 0;
long firstOffTimeInMinutes = 4*60;
long OnTimeInMinutes = 20*60;
long totalTime = 0;

Timer t;
Timeout tout;
short timerOverflow = 80;
uint16_t command = 0;
uint16_t bleData = 0;
long minutes = 0;

double fadeCounter = 0;

uint8_t pulseChannel = 0;

uint32_t count;

static volatile uint8_t write_flag=0;
#define FILE_ID     0x1111
#define REC_KEY     0x2222
static uint8_t m_data[2] = {setEffect,Abrightness};
fds_record_t        record;
fds_record_desc_t   record_desc;
fds_record_chunk_t  record_chunk;
fds_flash_record_t  flash_record;
fds_find_token_t    ftok;
uint32_t err_code;

uint8_t loop_effect = 0;
uint8_t loop_effect_array[32];

uint8_t brightness_pulse_one = Abrightness;
uint8_t brightness_pulse_two = 0;
int change_polarity = 0;

uint16_t combo_effects[10][2] = {{6,0},{6,0},{6,0},{6,0},{6,0},{6,0},{6,0},{6,0},{6,0},{6,0}};
uint8_t combo_index = 0;
uint8_t combo_effect = 0;
long seconds = 0;
long wait_for = 0;

int fade_a = 10;
int fade_c = 20;

int pulse_c = 10;
int pulse_a = -20;

unsigned concatenate(unsigned x, unsigned y) {
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;        
}

static ret_code_t fds_test_find_and_delete (void)
{
	ftok.page=0;
	ftok.p_addr=NULL;

	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	// Loop and find records with same ID and rec key and mark them as deleted. 
	while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
	{
		fds_record_delete(&record_desc);
		//NRF_LOG_PRINTF("Deleted record ID: %d \r\n",record_desc.record_id);
	//	// pc.printf("%s%d\n", "Deleted record ID: ",record_desc.record_id);
	}
	// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
	ret_code_t ret = fds_gc();
	if (ret != FDS_SUCCESS)
	{
			return ret;
	}
	return NRF_SUCCESS;
}

static ret_code_t fds_test_find_and_read (void)
{
	uint8_t *data;
	ftok.page=0;
	ftok.p_addr=NULL;

	memset(&ftok, 0x00, sizeof(fds_find_token_t));
	// Loop and find records with same ID and rec key and mark them as deleted. 
	while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
	{
		// // pc.printf("%s%d\n", "Opening record ID ", record_desc.record_id);
		err_code = fds_record_open(&record_desc, &flash_record);
		if ( err_code != FDS_SUCCESS)
		{
			return err_code;		
		}
		
	//	NRF_LOG_PRINTF("Found Record ID = %d\r\n",record_desc.record_id);
		// // pc.printf("Data = ");
		data = (uint8_t *) flash_record.p_data;
		for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
		{
			// // pc.printf("%d\n", data[i]);
			m_data[i] = data[i];
			//NRF_LOG_PRINTF("0x%8x ",data[i]);
		}
		// // pc.printf("\r\n");
		// Access the record through the flash_record structure.
		// Close the record when done.
		err_code = fds_record_close(&record_desc);
		if (err_code != FDS_SUCCESS)
		{
			return err_code;	
		}
	}
	// call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration

	return NRF_SUCCESS;
}


static ret_code_t fds_test_write(void)
{
	  //  // // pc.printf("%s\n", "ready to write to flash");
		// Set up data.
		record_chunk.p_data         = m_data;
		record_chunk.length_words   = 2;
		// Set up record.
		record.file_id              = FILE_ID;
		record.key              	= REC_KEY;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != FDS_SUCCESS)
		{
			// // pc.printf("%s\n", "Error writing record");
			return ret;
		}
		 // // pc.printf("%s%d \r\n", "Writing Record ID ", record_desc.record_id);
		 //NRF_LOG_PRINTF("Writing Record ID = %d \r\n",record_desc.record_id);
		return NRF_SUCCESS;
}

static ret_code_t fds_test_update(void)
{
	  //  // // pc.printf("%s\n", "ready to write to flash");
		// Set up data.
		
		// // pc.printf("%s%d\n", "data 0 : ", m_data[0]);
		record_chunk.p_data         = m_data;
		record_chunk.length_words   = 2;
		// Set up record.
		record.file_id              = FILE_ID;
		record.key              	= REC_KEY;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
				
		ret_code_t ret = fds_record_update(&record_desc, &record);

		if (ret != FDS_SUCCESS)
		{
			// // pc.printf("%s\n", "Error updating record");
			return ret;
		}
		 // // pc.printf("%s%d \r\n", "Updating Record ID ", record_desc.record_id);
		 //NRF_LOG_PRINTF("Writing Record ID = %d \r\n",record_desc.record_id);
		 // call the garbage collector to empty them, don't need to do this all the time, this is just for demonstration
		ret = fds_gc();
		if (ret != FDS_SUCCESS)
		{
			return ret;
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_read(void)
{
		//fds_find_token_t    ftok ={0};//Important, make sure you zero init the ftok token
		uint32_t *data;
		uint32_t err_code;

		ftok.page=0;
		ftok.p_addr=NULL;

		memset(&ftok, 0x00, sizeof(fds_find_token_t));
		
		// // pc.printf("Start searching... \r\n");
		// Loop until all records with the given key and file ID have been found.
		//while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS) {
				 err_code = fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok);
				 if ( err_code != FDS_SUCCESS)
				 {	
				 	// // pc.printf("%s\n", "Record not found");
				 	return err_code;		
				 }
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)
				{
					return err_code;		
				}
				
			//	NRF_LOG_PRINTF("Found Record ID = %d\r\n",record_desc.record_id);
				// // pc.printf("Data = ");
				data = (uint32_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
				{
					// // pc.printf("%d\n", data[i]);
					//NRF_LOG_PRINTF("0x%8x ",data[i]);
				}
				// // pc.printf("\r\n");
				// Access the record through the flash_record structure.
				// Close the record when done.
				err_code = fds_record_close(&record_desc);
				if (err_code != FDS_SUCCESS)
				{
					return err_code;	
				}
		//}
		return NRF_SUCCESS;
		
}

void initEffect(){

	CTRL_A.period_us(20);                   		//100Hz frequency
    CTRL_A.pulsewidth(18);
    CTRL_A = Abrightness * 0.01;
    wait(0.001);
    CTRL_B.period_us(20);
    CTRL_B.pulsewidth(18);
    CTRL_B = 0;

    PWM_CH1 = 1;
    PWM_CH2 = 0;

}

void executeCommand(){

	initEffect();
	
    //COMMNAD SET EFFECTS !AB#
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x42) {
    
        if(bleData <= 7){
            setEffect = bleData;
           // setPStorage(setEffect, Abrightness, Bbrightness);
        }
        if(bleData == COMBO_EFFECT){
        	seconds = 0;
        	combo_index = 0;
        }
    }
    //COMMAND SET SCHEDULE ON TIME IN MINUTES !AC###
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x43) {
        firstOffTimeInMinutes = bleData;
    }
    
    //COMMAND SET SCHEDULE OFF TIME IN MINUTES !AD
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x44) {
        OnTimeInMinutes = bleData;
    }
    
    //COMMAND START SCHEDULE TIMER !AE
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x45) {
        minutes = 0;
        scheduleFlag = 1;
        scheduleEffect = setEffect;
        seffect = setEffect;
        setEffect = 6;
    //    led1 = 0;
        setPeriod = 1;
    }
    
    //COMMAND STOP SCHEDULE TIMER  !AF
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x46) {
        scheduleFlag = 0;
   //     led1 = 1;
    }
    
    //COMMAND SET CHANNEL A BRIGHTNESS !AG###
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x47) {
        Abrightness = bleData;
        if (setEffect == 3)
        {
        	brightness_pulse_one = Abrightness;
			brightness_pulse_two = 0;
		    change_polarity = 0;
        }
      //  setPStorage(setEffect, Abrightness, Bbrightness);
    }
    
    //COMMAND SET CHANNEL A BRIGHTNESS  !AH
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x48) {
        Bbrightness = bleData;
      //  setPStorage(setEffect, Abrightness, Bbrightness);
    }
    
    //COMMAND SET CHANNEL A/B BRIGHTNESS  !AI
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x49) {
        Abrightness = bleData;
        Bbrightness = bleData;
      //  setPStorage(setEffect, Abrightness, Bbrightness);
    }
    //COMMAND SET CLEAR MEMORY  !AJ
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x4A){
    	err_code = fds_test_find_and_delete();
		// pc.printf("%s%d\n","error code", err_code);
		err_code = fds_test_write();
    }
    //COMMAND SET CLEAR MEMORY  !AK
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x4B){
    	NVIC_SystemReset();
    }

    /*******************COMBO************************************/
    //SET COMBO INDEX !CA# (1st)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x41){
    	combo_index = bleData;
    }
    //SET COMBO EFFECT !CB# (2nd)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x42){
    	combo_effect = bleData;
    }
    //SET COMBO TIME !CC### (3rd)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x43){
    	wait_for = bleData;
    	combo_effects[combo_index][0] = combo_effect;
		combo_effects[combo_index][1] = wait_for;
    }
}

void onDataWritten(const GattWriteCallbackParams *params)
{
    bleData = 0;
    if ((uart != NULL) && (params->handle == uart->getTXCharacteristicHandle())) {
        uint16_t bytesRead = params->len;
        
        if(params->data[0] == 0x21){
            command = (params->data[1] << 8 )|( params->data[2] );
            uint8_t i = 3;
            while(i < bytesRead){
                bleData = concatenate(bleData, (uint8_t)params->data[i] - 48);
                i++;
            }
            executeCommand();
        } else if (params->data[0] == 0x23){
        	command = (params->data[1] << 8 )|( params->data[2] );
            uint8_t j = 3;
            while(bytesRead > j){
                loop_effect_array[j] = (uint8_t)params->data[j]-48;
                j++;
            }
            executeCommand();
        }
        ble.updateCharacteristicValue(uart->getRXCharacteristicHandle(), params->data, bytesRead);
    }
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    ble.startAdvertising();
}

void flash(){
	double i = Abrightness;

	CTRL_B = 0;
	PWM_CH1 = 1;
	PWM_CH2 = 0;
	while(i < 1){
		CTRL_A = i;
		i += 0.01;
		wait_ms(1);	
	}
	while(i > 0){
		CTRL_A = i;
		i -= 0.01;
		wait_ms(1);
	}

	wait_us(500);

	i = 0;
	CTRL_A = 0;
	PWM_CH2 = 1;
	PWM_CH1 = 0;
	while(i < 1){
		CTRL_B = i;
		i += 0.01;
		wait_ms(1);	
	}
	while(i > 0){
		CTRL_B = i;
		i -= 0.01;
		wait_ms(1);
	}

	wait_us(500);
	         
}

void twinkle (){
/*
	double i = Abrightness * 0.01;
	
	CTRL_B = 0;
	PWM_CH1 = 1;
	CTRL_A = i;
	PWM_CH2 = 0;
	wait_ms(100);
	CTRL_B = i;
	PWM_CH1 = 0;
	CTRL_A = 0;
	PWM_CH2 = 1;
	wait_ms(400);
	CTRL_B = i;
	PWM_CH1 = 0;
	CTRL_A = 0;
	PWM_CH2 = 1;
	wait_ms(100);
	CTRL_B = i;
	PWM_CH1 = 0;
	CTRL_A = 0;
	PWM_CH2 = 1;
	wait_ms(100);

    wait(twinkledelayfour);
 */

		PWM_CH1 = 0;
		wait_ms(rand() % 75 + 125);
		CTRL_B = 0.01 * ( rand() % 10 + 100 );
		wait_ms(rand() % 75 + 125);
		CTRL_B = 0;
		wait_ms(rand() % 75 + 125);
		PWM_CH1 = 1;
		wait_ms(rand() % 75 + 125);

		PWM_CH2 = 0;
		wait_ms(rand() % 75 + 125);
		CTRL_A = 0.01 * ( rand() % 10 + 100 );
		wait_ms(rand() % 75 + 125);
		CTRL_A = 0;
		wait_ms(rand() % 75 + 125);
		PWM_CH2 = 1;
		wait_ms(rand() % 75 + 125);

}

void pulse(){
/*
	double i = Abrightness * 0.01;

	PWM_CH1 = 0;
	CTRL_B = 0.01 * pulse_c;
	wait_ms(1);
	CTRL_B = 0;
	PWM_CH1 = 1;
	wait_ms(1);

	PWM_CH2 = 0;
	CTRL_A = 0.01 * pulse_c;
	wait_ms(1);
	CTRL_A = 0;
	PWM_CH2 = 1;
	wait_ms(1);

	pulse_c += pulse_a;
	if(pulse_c < 10 || pulse_c > i){
		pulse_a = -pulse_a;
		pulse_c += pulse_a;
		if(pulse_a > 0){
			pulse_a = rand() % 20 + 80;
		}
	}
	*/

// Third version

	double i = Abrightness;
 	
	if (brightness_pulse_two < Abrightness && brightness_pulse_one <= Abrightness && change_polarity == 0){
		brightness_pulse_one -= 1;
		brightness_pulse_two += 1;
	} else {
		change_polarity = 1;
	}

	if (brightness_pulse_two >= 0 && brightness_pulse_one < Abrightness && change_polarity == 1){
		brightness_pulse_one += 1;
		brightness_pulse_two -= 1;	
	} else {
		change_polarity = 0;
	}
	int k = 0;
	do {
	    //j11 = brightness_pulse_one
	    CTRL_A = (CTRL_A == 0)?brightness_pulse_two * 0.01:0;
	    PWM_CH2 = (PWM_CH2 == 0)?1:0;
	    wait_us(200);
	    //j10 = brightness_pulse_two
	    CTRL_B = (CTRL_B == 0)?brightness_pulse_one * 0.01:0;
	    PWM_CH1 = (PWM_CH1 == 0)?1:0;
	    if(k < 31){
	    	wait_us(195);	
	    } else {
	    	wait_us(180);
	    }
	    
	    k++;
	} while (k < 32);

/*
	//first version

	double i = Abrightness * 0.01;
	long j = 3000;
	long double r = i / j;
	long double h = 0;


	if(ch == 0){

		if(PWM_CH1 == 1){
			PWM_CH1 = 0;
		} else {
			PWM_CH1 = 1;
		}

		if(PWM_CH2 == 1){
			PWM_CH2 = 0;
		} else {
			PWM_CH2 = 1;
		}

		CTRL_B = 0;

		while(j > 0) {
			
			CTRL_A = h;

			j--;

			h += r;

			wait_us(200);

		}

		while(j < 3000){

			CTRL_A = h;

			j++;
			
			h -= r;

			wait_us(200);

		}

	//	wait_ms(100);

	} else {

		if(PWM_CH1 == 1){
			PWM_CH1 = 0;
		} else {
			PWM_CH1 = 1;
		}

		if(PWM_CH2 == 1){
			PWM_CH2 = 0;
		} else {
			PWM_CH2 = 1;
		}

		CTRL_A = 0;

		while(j > 0) {
			
			CTRL_B = h;

			j--;

			h += r;

			wait_us(200);

		}

		while(j < 3000){

			CTRL_B = h;

			j++;
			
			h -= r;

			wait_us(200);

		}

	}
	*/

}

void fade(){
/*
    double r = fade_c * 0.01;
    double i = Abrightness * 0.01;

	if(PWM_CH1 == 1){
		PWM_CH1 = 0;
		CTRL_A = 0;
	} else {
		PWM_CH1 = 1;
		CTRL_A = i;
	}

	wait_ms(5);

	if(PWM_CH2 == 1){
		PWM_CH2 = 0;
		CTRL_B = 0;
	} else {
		PWM_CH2 = 1;
		CTRL_B = i;
	}

	wait_ms(5);

	fade_c += fade_a;

	if(fade_c < 20 || fade_c > 90) {
		fade_a = -fade_a;
	}
*/

	double i = Abrightness * 0.01;
	int j = 1000;
	double r = i / j;

	while(j > 0){
		if(PWM_CH1 == 1){
			PWM_CH1 = 0;
			CTRL_A = 0;
		} else {
			PWM_CH1 = 1;
			CTRL_A = i;
		}

		if(PWM_CH2 == 1){
			PWM_CH2 = 0;
			CTRL_B = 0;
		} else {
			PWM_CH2 = 1;
			CTRL_B = i;
		}

		j--;

		i -= r;
		wait_us(400);

	}
	while(j < 1000){

		if(PWM_CH1 == 1){
			PWM_CH1 = 0;
			CTRL_A = 0;
		} else {
			PWM_CH1 = 1;
			CTRL_A = i;
		}

		if(PWM_CH2 == 1){
			PWM_CH2 = 0;
			CTRL_B = 0;
		} else {
			PWM_CH2 = 1;
			CTRL_B = i;
		}

		j++;

		i += r;
		wait_us(400);
	}
	
}

void steadyon(){
/*
	double i = Abrightness * 0.01;

	PWM_CH1 = 0;   				// insures 2nd light off @end of burn time from previous loop               
	wait_ms(1);              
	CTRL_B = i;  				// turn first light on              
	wait_ms(4);                	// burn first light for 16ms              
	CTRL_B = 0;  				// then turn first light off               
	wait_ms(1);              
	PWM_CH1 = 1;  				// and turn on 2nd light              
	wait_ms(4);   				// burn 2nd light for 16ms

	PWM_CH2 = 0;   				// insures 2nd light off @end of burn time from previous loop               
	wait_ms(1);              
	CTRL_A = i;  				// turn first light on              
	wait_ms(4);                	// burn first light for 16ms              
	CTRL_A = 0;  				// then turn first light off               
	wait_ms(1);              
	PWM_CH2 = 1;  				// and turn on 2nd light              
	wait_ms(4);   				// burn 2nd light for 16ms
*/
	double i = Abrightness * 0.01;

    CTRL_B = (CTRL_B == 0)?i:0;
    PWM_CH1 = (PWM_CH1 == 0)?1:0;
    wait_us(800);
    CTRL_A = (CTRL_A == 0)?i:0;
    PWM_CH2 = (PWM_CH2 == 0)?1:0;
    wait_us(600);
}

void steadyoff(){
    PWM_CH1 = 1;
    PWM_CH2 = 1;
    CTRL_A = 0;
    CTRL_B = 0;
    SYS_CE = 1;    
}

void combo(){
	int localEffect = 0;
	localEffect = combo_effects[combo_index][0];
	wait_for = combo_effects[combo_index][1];
	
	if (seconds < wait_for){
		if(localEffect == 0){
			steadyon();
		} else if(localEffect == 1){
	        twinkle();   
	    } else if(localEffect == 2){
	        fade();
	    } else if(localEffect == PULSE_EFFECT){
	        pulse();
	    } else if(localEffect == 6){
	        steadyoff();
	    }
	} else {
		if(wait_for > 0){
			initEffect();
		}
		seconds = 0;
		combo_index++;
		if(combo_index >= 10){
			combo_index = 0;
		}
		
	}

}

void play(void){
    short effect = setEffect;
    if(effect == 0){
        steadyon();  
    } else if(effect == 1){
        twinkle();   
    } else if(effect == 2){
        fade();
    } else if(effect == PULSE_EFFECT){
        pulse();
    } else if(effect == 4){
        flash();
    } else if(effect == COMBO_EFFECT){
        combo();
    } else if(effect == 6){
        steadyoff();
    }
}

void keyReleased(void){
    if(setPeriod == 0){
        initEffect();
        setEffect++;
        if(setEffect > 6){
            SYS_CE = 1;
            setEffect = 0;
         //   setPStorage(setEffect, Abrightness, Bbrightness);
        }
        m_data[0] = setEffect;
        m_data[1] = Abrightness;
        
        fds_test_update();
      //  wait(0.5);
     //	fds_test_find_and_read();
    }
}

void keyPressed(void){
     setPeriod = 0;
}


void scheduleTimeOld(){
    if (scheduleFlag){
    	if(minutes <= 1){
	    	lbright = Abrightness;
	    	leffect = loop_effect;
	    	seffect = setEffect;
    	}
       if(firstOffTimeInMinutes >= minutes ){
   	
       	loop_effect = leffect;
       	setEffect = seffect;
       	Abrightness = lbright;
    
       } else {
         totalTime = firstOffTimeInMinutes + OnTimeInMinutes;
         if(totalTime >= minutes){
         	 loop_effect = 0;
         	 Abrightness = 0;
             setEffect = 6;
         } else {
            minutes = 0;
            Abrightness = lbright;
            loop_effect = leffect;
            setEffect = seffect;
         }
       }
    }    
}

void scheduleTime(){
	if (scheduleFlag)
	{
		if (firstOffTimeInMinutes >= minutes)
		{
			setEffect = 6;
		} else {
			totalTime = firstOffTimeInMinutes + OnTimeInMinutes;
			if(totalTime >= minutes)
			{
				setEffect = seffect;
			} else 
			{	
				firstOffTimeInMinutes = 24*60*60;
				minutes = 0;
			}
		}

	}
}

void updateMinutes(void){
    minutes++;
    scheduleTime();
}

void updateSeconds(void){
	seconds++;
}

void inputCheck(){
     
     while(!PWR_DET){  //!bt1
            counter++;
            wait(0.01); 
            if(counter > timerOverflow){
                minutes = 0;
                scheduleFlag = 1;
                scheduleEffect = setEffect;
            //    led1 = 0;
                setPeriod = 1;
            } else {
                scheduleFlag = 0;
            //    led1 = 1;    
            }
        } 
        counter = 0;
}

/* testing code fds */

static void my_fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_fds_evt->result != FDS_SUCCESS)
            {
                // Initialization failed.
            }
            break;
				case FDS_EVT_WRITE:
						if (p_fds_evt->result == FDS_SUCCESS)
						{
							// pc.printf("%s\n", "Whatever thing was doing is done");
							write_flag=1;
						}
						break;
        default:
            break;
    }
}

static ret_code_t fds_test_init (void)
{
	ret_code_t ret = fds_register(my_fds_evt_handler);

	if (ret != FDS_SUCCESS)
	{
		// pc.printf("%s\n", "not registered");
		return ret;
	}

	wait(0.5);

	ret = fds_init();

	if (ret != FDS_SUCCESS)
	{
		// pc.printf("%s\n", "not initilized");
		return ret;
	}
	
	return NRF_SUCCESS;
		
}

/* testing code end */

int main()
{
//	led1 = 0; //led2 = 0;
    SYS_CE = 0;
    initEffect();

    BT_PWR_DET.fall(&keyPressed);
    BT_PWR_DET.rise(&keyReleased);

    Ticker tick;
    tick.attach(updateMinutes, 60);

    Ticker tick2;
   	tick2.attach(updateSeconds, 1);

   	ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onDataWritten(onDataWritten);
    
    uart = new UARTService(ble);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"GLOWB", sizeof("GLOWB") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

	err_code = fds_test_init();
	// pc.printf("%s%d\n","error code", err_code);
	wait(0.5);

	ftok.page=0;
	ftok.p_addr=NULL;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));

	err_code = fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok);
	if ( err_code != FDS_SUCCESS)
	{
		err_code = fds_test_find_and_delete();
		// pc.printf("%s%d\n","error code", err_code);
		wait(0.5);
	 	err_code =fds_test_write();		
	} else {
		err_code = fds_test_find_and_read();
		setEffect = m_data[0];
		Abrightness = m_data[1];
	}

	//wait until the write is finished. 
	//while (write_flag==0);

    ble.setAdvertisingInterval(1000); /* 100ms; in multiples of 0.625ms. */
    ble.startAdvertising();

    while(true){
    	ble.waitForEvent();
    	inputCheck();
    	if(setEffect == COMBO_EFFECT){
    		combo();
    	} else {
    		play();
    	}
    }

    return 0;
}
