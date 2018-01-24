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
#include "DebouncedIn.h"

#define NEED_CONSOLE_OUTPUT 0 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */
#define ON_EFFECT 		0
#define TWINKLE_EFFECT 	1
#define FADE_EFFECT 	2
#define PULSE_EFFECT 	3
#define OFF_EFFECT 		4
#define COMBO_EFFECT 	5

#if NEED_CONSOLE_OUTPUT
#define DEBUG(STR) { if (uart) uart->write(STR, strlen(STR)); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define NUM_PAGES 4

//DebouncedIn PWR_DET(p19);
DigitalIn READ_DET(p19);
InterruptIn PWR_DET(p19);
DigitalOut SYS_CE(p26);          			//26 - P0.26 and P0.27 are by default used for the 32 kHz crystal and are not available on the connectors
DigitalOut PWM_CH1(p12);                    //12
DigitalOut PWM_CH2(p13);					//13
PwmOut CTRL_A(p18);             			//18
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
uint32_t bleData = 0;
long minutes = 1511388305;
long tmp_minutes = 0;
long start_schedule = 0;
long end_schedule = 4*60*60;

//james variables
bool ready_flag = false;
//end

uint32_t count;

static volatile uint8_t write_flag=0;
#define FILE_ID     0x1111
#define REC_KEY     0x2222
static uint8_t m_data[22] = {setEffect,Abrightness,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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

uint16_t combo_effects[10][2] = {{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}};
uint8_t combo_index = 0;
uint8_t combo_effect = 0;
long local_wait_for = 0;
int local_combo_index = 0;
int local_combo_effect = 0;
uint32_t seconds = 0;
long wait_for = 0;

int fade_a = 10;
int fade_c = 20;

int pulse_c = 10;
int pulse_a = -20;
short pulse_brightness = 0;

int interrupted = 0;

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
	while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
	{
		fds_record_delete(&record_desc);
	}

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
	while (fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok) == FDS_SUCCESS)
	{
		err_code = fds_record_open(&record_desc, &flash_record);
		if ( err_code != FDS_SUCCESS)
		{
			return err_code;		
		}
		data = (uint8_t *) flash_record.p_data;
		for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
		{
			m_data[i] = data[i];
		}
	
		err_code = fds_record_close(&record_desc);
		if (err_code != FDS_SUCCESS)
		{
			return err_code;	
		}
	}
	
	return NRF_SUCCESS;
}


static ret_code_t fds_test_write(void)
{
		record_chunk.p_data         = m_data;
		record_chunk.length_words   = 22;

		record.file_id              = FILE_ID;
		record.key              	= REC_KEY;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
				
		ret_code_t ret = fds_record_write(&record_desc, &record);
		if (ret != FDS_SUCCESS)
		{
			return ret;
		}
	
		return NRF_SUCCESS;
}

static ret_code_t fds_test_update(void)
{
		record_chunk.p_data         = m_data;
		record_chunk.length_words   = 22;

		record.file_id              = FILE_ID;
		record.key              	= REC_KEY;
		record.data.p_chunks        = &record_chunk;
		record.data.num_chunks      = 1;
				
		ret_code_t ret = fds_record_update(&record_desc, &record);

		if (ret != FDS_SUCCESS)
		{
			return ret;
		}
		 
		ret = fds_gc();
		if (ret != FDS_SUCCESS)
		{
			return ret;
		}
		return NRF_SUCCESS;
}

static ret_code_t fds_read(void)
{
		uint32_t *data;
		uint32_t err_code;

		ftok.page=0;
		ftok.p_addr=NULL;

		memset(&ftok, 0x00, sizeof(fds_find_token_t));

				 err_code = fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok);
				 if ( err_code != FDS_SUCCESS)
				 {	
				 	return err_code;		
				 }
				err_code = fds_record_open(&record_desc, &flash_record);
				if ( err_code != FDS_SUCCESS)
				{
					return err_code;		
				}
				
				data = (uint32_t *) flash_record.p_data;
				for (uint8_t i=0;i<flash_record.p_header->tl.length_words;i++)
				{
				
				}
				
				err_code = fds_record_close(&record_desc);
				if (err_code != FDS_SUCCESS)
				{
					return err_code;	
				}
		return NRF_SUCCESS;
		
}

void initEffect(){
    CTRL_A = 0;
    CTRL_B = 0;
    PWM_CH1 = 1;
    PWM_CH2 = 1;
}

void executeCommand(){

	initEffect();
    //COMMNAD SET EFFECTS !AB#
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x42) {
    
        if(bleData <= OFF_EFFECT){
            setEffect = bleData;
        }
        if(bleData == COMBO_EFFECT){
        	setEffect = COMBO_EFFECT;
        	seconds = 0;
        	combo_index = 0;
        }

        m_data[0] = setEffect;
        m_data[1] = Abrightness;
        
        fds_test_write();
    }
    //COMMAND SET SCHEDULE ON TIME IN SECONDS !AL##########
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x4C) {
    	minutes = bleData;
    	set_time(minutes);
    }
    //COMMAND SET SCHEDULE ON TIME IN SECONDS !AC##########
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x43) {
    	start_schedule = bleData;
    }
    
    //COMMAND SET SCHEDULE OFF TIME IN SECONDS !AD##########
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x44) {
    	end_schedule = bleData;
    }
    
    //COMMAND START SCHEDULE TIMER !AE
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x45) {
        scheduleFlag = 1;
        scheduleEffect = setEffect;
        seffect = setEffect;
        setEffect = OFF_EFFECT;
 
        setPeriod = 1;
    }
    
    //COMMAND STOP SCHEDULE TIMER  !AF
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x46) {
        scheduleFlag = 0;
    }
    
    //COMMAND SET CHANNEL A BRIGHTNESS !AG###
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x47) {
        Abrightness = bleData;
        m_data[0] = setEffect;
        m_data[1] = Abrightness;
        
        fds_test_write();
    }
    
    //COMMAND SET CHANNEL A BRIGHTNESS  !AH - NOT USED
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x48) {
        Bbrightness = bleData;
    }
    
    //COMMAND SET CHANNEL A/B BRIGHTNESS  !AI - NOT USED
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x49) {
        Abrightness = bleData;
        Bbrightness = bleData;
    }
    //COMMAND SET CLEAR MEMORY  !AJ
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x4A){
    	err_code = fds_test_find_and_delete();
		err_code = fds_test_write();
    }
    //COMMAND SET CLEAR MEMORY  !AK
    if((command & 0xFF00) == 0x4100 && (command & 0xFF) == 0x4B){
    	NVIC_SystemReset();
    }

    /*******************COMBO************************************/
    //SET COMBO INDEX !CA# (1st)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x41){
    	local_combo_index = bleData;
    }
    //SET COMBO EFFECT !CB# (2nd)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x42){
    	local_combo_effect = bleData;
    }
    //SET COMBO TIME !CC### (3rd)
    if((command & 0xFF00) == 0x4300 && (command & 0xFF) == 0x43){
    	local_wait_for = bleData;
    	combo_effects[local_combo_index][0] = local_combo_effect;
		combo_effects[local_combo_index][1] = local_wait_for;
		m_data[(local_combo_index+1)*2] = local_combo_effect;
		m_data[((local_combo_index+1)*2)+1] = local_wait_for;
		fds_test_write();
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

void twinkle (){

  if(pulse_brightness != Abrightness){
        pulse_brightness = Abrightness;
  }
  initEffect();                		        	 // insure all circuits open
  brightness_pulse_one = rand() % 75 +25;	 // brightness = random 25 to 100
  CTRL_A = brightness_pulse_one * 0.01;  	 // Switch A-side LEDs on
  PWM_CH2 = 0;			      	 // close A-side circuit
  wait_ms(rand() % 50 + 75);		      	 // Leave on (random 75-125ms 
  initEffect();                		      	 // insure all circuits open
  wait_ms(rand() % 100 + 25);		       	 // Stay off (random 25-125ms 
  brightness_pulse_one = rand() % 75 +25;
  CTRL_B = brightness_pulse_one * 0.01;	 // Switch A-side LEDs on
  PWM_CH1 = 0;				 // close A-side circuit
  wait_ms(rand() % 50 + 75);			 // Leave on (random 75-125ms 
  initEffect();                		      	 // insure all circuits open

}

void pulse(){

   int c = 1;		// counter;
   int a = -1;		// counter increment
   int steps = 100;		// resolution
   double i = 1;
   if(pulse_brightness != Abrightness){	// if brightness changed
        pulse_brightness = Abrightness; 	// reset to new max PWM
   }
   do{
        brightness_pulse_one = i * c;
        brightness_pulse_two = i * (steps - c);
        initEffect();
        CTRL_A = (CTRL_A == 0)?brightness_pulse_two * 0.01:0; //I added (CTRL_A == 0)? assuming that's what the code should look like
        PWM_CH2 = 0;
        wait_ms(6);
        initEffect();
        CTRL_B = (CTRL_B== 0)?brightness_pulse_one * 0.01:0; // I added (CTRL_B == 0)? assuming that's what the code should look like
        PWM_CH1 = 0;
        wait_ms(6);
        initEffect();
        c = c + a;
        if (c < 1||c > steps){
            a   = -a;
            c  =  c + a;
        }
        if (a   >  0){
            a   = rand() % 1 + 5;
        }
   } while (!(c < 2 && a < 0) && interrupted == 0);   //add code to break loop if button
        				// pressed or combo effect time expire
}

void fade(){

	double i = Abrightness * 0.01;	// brightness
    int j = 1000;				// counter
    int a = 2;				// counter increment
    double r = i / j; 			// 
    while(j <1001 && interrupted == 0){
        // We suggest - add code in condition above to break out on button press or combo effect timeout
        initEffect();     	// insure all circuits open
        CTRL_B = i;	  		// Apply brightess/PWM to B-side
        PWM_CH1 = 0;   		// Switch B-side LEDs on
        wait_ms(6);        	// Leave B-side LEDs on for 6 ms
        initEffect();
        
        CTRL_A = i; 		// Apply brightess/PWM to A-side
        PWM_CH2 = 0;   		// Switch A-side LEDs on
        wait_ms(6);         // Leave A-side LEDs on for 6 m
        initEffect();
        j = j - a;			// update counter
        i = j *  r;			// recalculate brightness
        if( j < 1){a = -a;} // reverse fade at end of first loop

   }
  
}

void steadyon(){

	double i = Abrightness * 0.01;
	//code suggested
	initEffect();
    CTRL_B = i;		
    PWM_CH1 = 0;
    wait_ms(6);	
    initEffect();		
    CTRL_A = i;		 
    PWM_CH2 = 0;
    wait_ms(6);  
    initEffect();
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
	
	if (wait_for > 0 && seconds < wait_for){
		if(localEffect == ON_EFFECT){
			steadyon();
		} else if(localEffect == TWINKLE_EFFECT){
	        twinkle();   
	    } else if(localEffect == FADE_EFFECT){
	        fade();
	    } else if(localEffect == PULSE_EFFECT){
	        pulse();
	    } else if(localEffect == OFF_EFFECT){
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
    if(effect == ON_EFFECT){
        steadyon();  
    } else if(effect == TWINKLE_EFFECT){
        twinkle();   
    } else if(effect == FADE_EFFECT){
        fade();
    } else if(effect == PULSE_EFFECT){
        pulse();
    } else if(effect == OFF_EFFECT){
        steadyoff();
    }
}

void keyPressed(void){
	interrupted = 1;
}

void keyReleased(void){
    if(setPeriod == 0){
        initEffect();
        setEffect++;
        if(setEffect > OFF_EFFECT){
            SYS_CE = 1;
            setEffect = ON_EFFECT;
        }
        m_data[0] = setEffect;
        m_data[1] = Abrightness;
        
        fds_test_write();
    }
}

void scheduleTime(){
	if(minutes >= start_schedule && minutes < end_schedule){
		setEffect = scheduleEffect;
	} else if(minutes == end_schedule) {
		start_schedule += 24*60*60;
		end_schedule += 24*60*60;
	} else {
		setEffect = OFF_EFFECT;
	}
}

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
		return ret;
	}

	wait(0.5);

	ret = fds_init();

	if (ret != FDS_SUCCESS)
	{
		return ret;
	}
	
	return NRF_SUCCESS;
		
}

int main()
{
    SYS_CE = 0;
    PWR_DET.fall(&keyPressed);
    
    CTRL_A.period_ms(1);			//Frequency 1Khz ...
    CTRL_B.period_ms(1);			//Frequency 1Khz ...

    set_time(minutes);

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
	wait(0.5);

	ftok.page=0;
	ftok.p_addr=NULL;
	memset(&ftok, 0x00, sizeof(fds_find_token_t));

	err_code = fds_record_find(FILE_ID, REC_KEY, &record_desc, &ftok);
	if ( err_code != FDS_SUCCESS)
	{
		err_code = fds_test_find_and_delete();
		wait(0.5);
	 	err_code =fds_test_write();		
	} else {
		err_code = fds_test_find_and_read();
		setEffect = m_data[0];
		Abrightness = m_data[1];
		combo_effects[0][0] = m_data[2];
		combo_effects[0][1] = m_data[3];
		combo_effects[1][0] = m_data[4];
		combo_effects[1][1] = m_data[5];
		combo_effects[2][0] = m_data[6];
		combo_effects[2][1] = m_data[7];
		combo_effects[3][0] = m_data[8];
		combo_effects[3][1] = m_data[9];
		combo_effects[4][0] = m_data[10];
		combo_effects[4][1] = m_data[11];
		combo_effects[5][0] = m_data[12];
		combo_effects[5][1] = m_data[13];
		combo_effects[6][0] = m_data[14];
		combo_effects[6][1] = m_data[15];
		combo_effects[7][0] = m_data[16];
		combo_effects[7][1] = m_data[17];
		combo_effects[8][0] = m_data[18];
		combo_effects[8][1] = m_data[19];
		combo_effects[9][0] = m_data[20];
		combo_effects[9][1] = m_data[21];
	}

    ble.setAdvertisingInterval(5000); /* 100ms; in multiples of 0.625ms. */
    ble.startAdvertising();
    initEffect();
    int init = 0;
 
    while(true){
    	ble.waitForEvent();
    	if(interrupted == 1){
    		interrupted = 0;
			while(!READ_DET.read()){
				init++;
	    		if(init <= 1){
					setEffect++;
					if(setEffect > OFF_EFFECT){
			            SYS_CE = 1;
			            setEffect = ON_EFFECT;
		         	}
				} else if(init > 1 && init <= 5){
					//this will be change to auto time or something
					scheduleFlag = 1;
			        scheduleEffect = setEffect;
			        seffect = setEffect;
			        setEffect = OFF_EFFECT;
			        setPeriod = 1;
				} else if(init > 5) {
					setEffect = OFF_EFFECT;
					initEffect();
					steadyoff();
				}
				wait(1);
			}
		}

		if(init > 0){
			m_data[0] = setEffect;
			fds_test_write();
			init = 0;
			if(setEffect != OFF_EFFECT){
				initEffect();
			}
		}
			
        time_t tmp_minutes = time(NULL); //this are seconds - miss-named for minutes for previous code
        if(scheduleFlag || setEffect == COMBO_EFFECT){
        	if(minutes != tmp_minutes){
        		minutes = tmp_minutes;
        		seconds++;
        	}
        }
    	if(setEffect == COMBO_EFFECT){
    		combo();
    	} else {
    		play();
    	}
    }

    return 0;
}
