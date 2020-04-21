
//#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "ili9341.h"
#include "ili9341gfx.h"
#include <avr/pgmspace.h>
#include "grn_TWI.h"
#include <avr/interrupt.h>
#include "uart.h"
#include <string.h>
#include <avr/sleep.h>
#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"

#define DEBUG 1
#define DPS310_W 0xee
#define DPS310_R 0xef
#define PRS_B2	0x00
#define PRS_B1	0x01
#define PRS_B0	0x02
#define TMP_B2 	0x03
#define TMP_B1	0x04
#define TMP_B0	0x05
#define PRS_CFG	0x06
#define TMP_CFG	0x07
#define MEAS_CFG	0x08
#define CFG_REG		0x09
#define INT_STS		0x0A
#define FIFO_STS	0x0B
#define RESET		0x0C
#define PRODUCT_ID	0x0D
#define COEF_SRCE	0x28

#define COEF_RDY 1
#define SENSOR_RDY 2
#define TMP_RDY 3
#define PRS_RDY 4
#define PROD_ID 5

#define LOW 1
#define MID 2
#define HIGH 3
#define ULTRA 4

//Compensation Scale Factors (Oversampling)
#define Scal_1 524288 //sinlge
#define Scal_2 1572864
#define Scal_4 3670016
#define Scal_8 7864320
#define Scal_16 253952
#define Scal_32 516096
#define Scal_64 1040384
#define Scal_128 2088960

#define SENS_OP_STATUS 0x08

#define MODE_STBY	0x00
#define MODE_COMMAND_PRES 0x01
#define MODE_COMMAND_TEMP 0x02
#define MODE_COMMAND_PRES_AND_TEMP 0x03
#define MODE_BACKGROUND_PRES 0x05
#define MODE_BACKGROUND_TEMP 0x06
#define MODE_BACKGROUND_PRES_AND_TEMP 0x07
#define POINTCOLOUR PINK

#define SENSOR_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<6)) != 0
#define COEFF_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<7)) != 0
#define TEMP_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<5)) != 0
#define PRES_READY_CHECK (DPS310_read(MEAS_CFG) & (1<<4)) != 0

#define JOY_PUSH !(PINC & (1<<PC0)) && (entprell == 0)
#define JOY_HOR ReadADC(2)
#define JOY_VERT ReadADC(1)
#define LEFT > 600
#define RIGHT <400
#define DOWN > 600
#define UP <400

#define UPDATE_STATE state = sd_card("",CHECK)//check position in state machine
#define RELOAD_ENTPRELL 100

//SD Card Actions
#define NOT_USED 0
#define CLOSE 1
#define CLOSED 2
#define OPEN 3
#define OPENED 4 
#define WRITE 5
#define WRITING 6
#define SYNC 7
#define SYNCING 8
#define COMMUNICATE 9
#define COMMUNICATING 10
#define CHECK 11			//check Status
#define NIL 12 //no specific action

extern uint16_t vsetx,vsety,vactualx,vactualy,isetx,isety,iactualx,iactualy;
static FILE mydata = FDEV_SETUP_STREAM(ili9341_putchar_printf, NULL, _FDEV_SETUP_WRITE);
uint8_t result,  xpos, error, x, value, rdy;
uint16_t xx, yy, zell, COLOR, var_x,color;
uint8_t messung=1;

uint8_t sd_com; //flag sd card communication via uart on / off
char sd_string[40] = "";


uint8_t str_len=0;//length of string
	struct fat_file_struct* fd;
	struct fat_dir_struct* dd;
	struct fat_fs_struct* fs;
	struct partition_struct* partition;




//compensation coefficients
int16_t m_C0;
int16_t m_C1;
int32_t m_C00;
int32_t m_C10;
int16_t m_C01;
int16_t m_C11;
int16_t m_C20;
int16_t m_C21;
int16_t m_C30;

//uint8_t buffer[3] = {0};
uint8_t meas=0;
uint8_t id=0;
uint8_t pres_ovs, temp_ovs;

uint8_t ms, ms10,ms100,sec,min,entprell, state;

uint16_t tt=0;
double pp=0;
uint8_t buff[6]= {0};
//logging function
uint16_t log_pos;


long Pressure;
uint16_t Temperature;
uint32_t altitude;
uint32_t qnh;

uint8_t state; //status state machine

uint8_t messung, count;

uint16_t posx, posy;

uint8_t bit;
void init_ili9341(void);
// String für Zahlenausgabe
char string[30] = "";

//prototypen DPS310
uint8_t DPS310_read(uint8_t reg);
uint8_t DPS310_write(uint8_t reg, uint8_t data);
int16_t DPS310_readCoeffs(void);
void DPS310_sreset(void);

void DPS310_init(uint8_t acc);
uint32_t DPS310_get_sc_temp(uint8_t oversampling);
long DPS310_get_temp(uint8_t oversampling);
double DPS310_get_pres(uint8_t t_ovrs, uint8_t p_ovrs);

//Prototypen SD-Card
static uint8_t read_line(char* buffer, uint8_t buffer_length);
static uint32_t strtolong(const char* str);
static uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name); 
static uint8_t print_disk_info(const struct fat_fs_struct* fs);

ISR (TIMER1_COMPA_vect)
{
	ms10++;
	if(entprell != 0)entprell--;
	if(ms10==10)	//100ms
	{
		ms10=0;
		ms100++;
	}
    if(ms100==10)	//sekunde
	{
		ms100=0;
		sec++;
		messung=1;
	}
	if(sec==60)	//Minute
	{
		sec=0;
		min++;
		
	}
}
long calcalt(double press, uint32_t pressealevel);
uint32_t xxx;
uint16_t vor_komma(uint32_t value);
uint8_t nach_komma(uint32_t value);
uint16_t ReadADC(uint8_t ADCchannel);
void showADC(void);
void showSD(uint8_t stat);

int main(void)
{
	
	DDRC &= ~(1<<PC0); //TFT DC Pin
	PORTC |= (1<<PC0);//Pullup Activate
	
	DDRD |= (1<<PD6);//CS SD Card
	PORTD |= (1<<PD6);// High
	
	init_ili9341();
	uart_init();
	//display_init();//display initial data
	sd_com=0;
	yy=240;
	xx=0;
	zell=0;
	color=123;
	var_x=0x01;
	temp_ovs=0;
	pres_ovs=0;
	value=0;
	rdy=0;
	altitude=0;
	tt=0;
	log_pos=0;
	qnh=101525;
	uint8_t log=0;
	state=CLOSED;
	messung=0;
	count=0;
	
	
	
	TWIInit();
	//Timer 1 Configuration
	OCR1A = 0x009C;	//OCR1A = 0x3D08;==1sec
	TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match
    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer
    
   //ADC
    // Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC 
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
        
    sei();
    // enable interrupts
		
	DPS310_init(ULTRA);
	
	/* we will just use ordinary idle mode */
    set_sleep_mode(SLEEP_MODE_IDLE);
	
	 /* setup sd card slot */
				 if(!sd_raw_init())uart_puts_p(PSTR("MMC/SD initialization failed\n"));
    
				// open first partition 
				partition = partition_open(sd_raw_read, sd_raw_read_interval, sd_raw_write, sd_raw_write_interval, 0);
				 if(!partition)
				{
					/* If the partition did not open, assume the storage device
					 * is a "superfloppy", i.e. has no MBR.
					 */
					partition = partition_open(sd_raw_read, sd_raw_read_interval, sd_raw_write, sd_raw_write_interval, -1);
					if(!partition)uart_puts_p(PSTR("opening partition failed\n"));
				}
				 
				 /* open file system */
				fs = fat_open(partition);
				if(!fs)uart_puts_p(PSTR("opening filesystem failed\n"));

			 
			 	/* open root directory */
				struct fat_dir_entry_struct directory;
				fat_get_dir_entry_of_path(fs, "/", &directory);

				dd = fat_open_dir(fs, &directory);
				if(!dd)uart_puts_p(PSTR("opening root directory failed\n"));
			
				/* print some card information as a boot message */
				print_disk_info(fs);
						
				/* open file */		
				fd = open_file_in_dir(fs, dd, "data.txt");
				if(!fd)uart_puts("could not open data.txt\n");
				
				
				uart_puts("Initialisation success\n");
				
			
				
				
			
				///////////////////////////////////////////////////////////////////////////////////
	while(1)
	{
		
		if(messung)
		{	
			messung=0;	
			if(TEMP_READY_CHECK)
			{
				showADC();
				count++;		
				
				Temperature=DPS310_get_temp(temp_ovs);
				ili9341_setcursor(10,120);
				printf("T: %d C", Temperature);
				ili9341_setcursor(10,170);
				printf("T: %d.%d\370 C", vor_komma(Temperature), nach_komma(Temperature));
				ili9341_setcursor(10,190);
				printf("A: %d.%2.2d m", vor_komma(altitude), nach_komma(altitude));
				ili9341_setcursor(10,210);
				printf("P: %d.%1.2d hPa", vor_komma(Pressure), nach_komma(Pressure));
				altitude = calcalt(Pressure, qnh);
				
				if(log)
				{
					sprintf(string,"%d,\t ", count);
					str_len=strlen((const char *)string);
					fat_write_file(fd,(const uint8_t*)string,str_len);
					sd_raw_sync();
					uart_puts(string);
					sprintf(string,"Temperature: %d.%dC,\t ", vor_komma(Temperature), nach_komma(Temperature),count);
					str_len=strlen((const char *)string);
					fat_write_file(fd,(const uint8_t*)string,str_len);
					sd_raw_sync();
					uart_puts(string);
					sprintf(string,"Pressure (QNH): %d.%1.2d hPa \n", vor_komma(Pressure), nach_komma(Pressure));
					str_len=strlen((const char *)string);
					fat_write_file(fd,(const uint8_t*)string,str_len);
					sd_raw_sync();
					uart_puts(string);
					
				}
			}
						
			
		}//end of messung
						
			if(PRES_READY_CHECK)
			{
				Pressure=DPS310_get_pres(temp_ovs, pres_ovs);
				tt++;
			}
			if((JOY_HOR RIGHT) && !entprell)
			{
				entprell = RELOAD_ENTPRELL;
				
				log=1  ;
			}
			
			if((JOY_HOR LEFT) && !entprell)
			{
				entprell = RELOAD_ENTPRELL;
				fat_close_file(fd);
				/* close directory */
				fat_close_dir(dd);
				/* close file system */
				fat_close(fs);
				/* close partition */
				partition_close(partition);
				uart_puts("SD is closed...\n");
				log=0;
			}
			

	}//end of while

}//end of main

uint8_t DPS310_read(uint8_t reg)
{
		uint8_t result=0;
		
		TWIStart();
		if(TWIGetStatus() != 0x08)return 123;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 2;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 3;
		TWIStart();
		if(TWIGetStatus() != 0x10)return 4; //repetet Start sent?
		TWIWrite(DPS310_R);
		if(TWIGetStatus() != 0x40)return 5;
		result=TWIReadNACK();
		TWIStop();
		_delay_us(30);
	return result;	
//Daten zurueckgeben
}
uint8_t DPS310_write(uint8_t reg, uint8_t data)
{
		TWIStart();
		if(TWIGetStatus() != 0x08)return 11;
		TWIWrite(DPS310_W);
		if(TWIGetStatus() != 0x18)return 22;
		TWIWrite(reg);
		if(TWIGetStatus() != 0x28)return 33;
		TWIWrite(data);
		if(TWIGetStatus() != 0x28)return 44;
		TWIStop();
		
		_delay_us(30);
	return 0;	
	
	//Daten zurueckgeben
}

int16_t DPS310_readCoeffs(void)
{
	uint16_t buffer[19];//coeffizienten
	uint8_t coeff_start;
	coeff_start=0x10;
	
	//coeffizienten einlesen und in buffer-Array speichern
	//Addressen 0x10 - 0x21
	for(x=0;x<18;x++)
	{
		buffer[x]=DPS310_read(coeff_start);
		_delay_ms(10);
		coeff_start++;
	}
	 
    m_C0=(((int)buffer[0]<<8)|buffer[1])>>4;
    m_C0=m_C0/2;
      
    m_C1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	if(m_C1 & ((uint32_t)1 << 11))
	{
		m_C1 -= (uint32_t)1 << 12;
	}
      
    m_C00= ((((long)buffer[3]<<8)|buffer[4])<<8)|buffer[5];
    m_C00=(m_C00<<8)>>12;

    m_C10=((((long)buffer[5]<<8)|buffer[6])<<8)|buffer[7];
    m_C10=(m_C10<<12)>>12;

    m_C01=((int)buffer[8]<<8)|buffer[9];

    m_C11=((int)buffer[10]<<8)|buffer[11];

    m_C20=((int)buffer[12]<<8)|buffer[13];

    m_C21=((int)buffer[14]<<8)|buffer[15];

    m_C30=((int)buffer[16]<<8)|buffer[17];
       
    return 0;
}


void DPS310_sreset(void)
{
	// softreset of DPS310 sensor
	DPS310_write(0x0c, 0x99);
	_delay_ms(50);
}

void init_ili9341(void)
{
	stdout = & mydata;
	ili9341_init();//initial driver setup to drive ili9341
	ili9341_clear(BLACK);//fill screen with black colour
	_delay_ms(100);
	ili9341_setRotation(3);//rotate screen
	_delay_ms(2);
	ili9341_settextcolour(YELLOW,BLACK);
	ili9341_setcursor(0,0);
	ili9341_settextsize(2);
}
void DPS310_init(uint8_t acc)
{
	uint8_t bit=0;
	
	while(bit==0)// go on if Sensor ready flag is set
	{
		if(COEFF_READY_CHECK)bit=1;
		DPS310_readCoeffs();
		
		switch(acc)
		{
			case LOW: 	DPS310_write(PRS_CFG, 0x00);//eight times low power
						DPS310_write(TMP_CFG, 0x80);// 1 measurement
						DPS310_write(CFG_REG, 0x00);//p bit shift off
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 1;
						break;
			case MID: 	DPS310_write(PRS_CFG, 0x14);//2 messungen / sek   16 fach ovs
						DPS310_write(TMP_CFG, 0x90);//externer sens  2 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x04);//p bit shift on
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 16;
						break;
			case HIGH: 	DPS310_write(PRS_CFG, 0x26);//4 messungen / sek   64 fach ovs
						DPS310_write(TMP_CFG, 0xA0);//externer sens  4 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x04);//p bit shift on 
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 1;
						pres_ovs = 64;
						break;
			case ULTRA:	DPS310_write(PRS_CFG, 0xF7);//4 messungen / sek   64 fach ovs
						DPS310_write(TMP_CFG, 0xD7);//externer sens  4 messung / sek  single ovs
						DPS310_write(CFG_REG, 0x0C);//p bit shift on 
						DPS310_write(MEAS_CFG, 0x07);//cont temp and pres mess
						temp_ovs = 128;
						pres_ovs = 128;
						break;
			
		}
		//Korrekturwerte für falsche Temperaturwerte (2-fach normaler Temp Wert)
		// Quelle: https://github.com/Infineon/DPS310-Pressure-Sensor
		
		DPS310_write(0x0E, 0xA5);
		DPS310_write(0x0F, 0x96);
		DPS310_write(0x62, 0x02);
		DPS310_write(0x0E, 0x00);
		DPS310_write(0x0F, 0x00);
	}
}
uint32_t DPS310_get_sc_temp(uint8_t oversampling)
{
	long temp_raw=0;

	buff[0] = DPS310_read(TMP_B2);
	buff[1] = DPS310_read(TMP_B1);
	buff[2] = DPS310_read(TMP_B0);
			
	temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
	temp_raw=(temp_raw<<8)>>8;
				
	return temp_raw; 
}

long DPS310_get_temp(uint8_t oversampling)
{
	long temp_raw=0;
	double temp_sc=0;
	double temp_comp=0;
	long scalfactor=0;

			buff[0] = DPS310_read(TMP_B2);
			buff[1] = DPS310_read(TMP_B1);
			buff[2] = DPS310_read(TMP_B0);
			
			temp_raw=DPS310_get_sc_temp(oversampling);
			
			switch(oversampling)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
			temp_sc = (float)temp_raw/scalfactor;
			temp_comp=m_C0+m_C1*temp_sc;
			
			
			return temp_comp*100; //2505 entspricht 25,5 Grad
}

double DPS310_get_pres(uint8_t t_ovrs, uint8_t p_ovrs)
{
	long temp_raw;
	double temp_sc;
	
	long prs_raw;
	double prs_sc;
	double prs_comp;
	long scalfactor=0;
	
		buff[0] = DPS310_read(TMP_B2);
		buff[1] = DPS310_read(TMP_B1);
		buff[2] = DPS310_read(TMP_B0);
		
		temp_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		temp_raw=(temp_raw<<8)>>8;
		
		switch(t_ovrs)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
			temp_sc = (float)temp_raw/scalfactor;
		
		buff[0] = DPS310_read(PRS_B2);
		buff[1] = DPS310_read(PRS_B1);
		buff[2] = DPS310_read(PRS_B0);
		
		prs_raw=((((long)buff[0]<<8)|buff[1])<<8)|buff[2];
		prs_raw=(prs_raw<<8)>>8;
		
		switch(p_ovrs)
			{
				case 1:	scalfactor = 524288;break;
				case 2:	scalfactor = 1572864;break;
				case 4:	scalfactor = 3670016;break;
				case 8:	scalfactor = 7864320;break;
				case 16:	scalfactor = 253952;break;
				case 32:	scalfactor = 516096;break;
				case 64:	scalfactor = 1040384;break;
				case 128:	scalfactor = 2088960;break;
				
			}
		prs_sc = (float)prs_raw/scalfactor;
		prs_comp=m_C00+prs_sc*(m_C10+prs_sc*(m_C20+(prs_sc*m_C30)))+temp_sc*m_C01+temp_sc*prs_sc*(m_C11+(prs_sc*m_C21));
		return prs_comp; //2505 entspricht 25,5 Grad
}

long calcalt(double press, uint32_t pressealevel)
{
   return 100*(44330 * (1 - pow((double) press / pressealevel, 0.1902226)));
	//*100 um stellen von Komma nicht zu verlieren
}

uint16_t vor_komma(uint32_t value)
{
	return value/100;
	
}
uint8_t nach_komma(uint32_t value)
{
	uint8_t temp;
	temp = value/100;
	return value-(temp*100);
	
	
}
uint16_t ReadADC(uint8_t ADCchannel)
{
 //select ADC channel with safety mask
 ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
 //single conversion mode
 ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );
 return ADC;
}
void showSD(uint8_t stat)
{
	//print SD Card Status
	ili9341_setcursor(90,0);
	printf("          ");
	switch(stat)
	{
		case CLOSED:	printf("SD Closed");
						break;
		case OPENED:	printf("SD Open");
						break;
		case WRITING:	printf("SD WRITING");
						break;
		case SYNCING:	printf("SD Syncing");
						break;
		default:		printf("?????");
						break;
	}
}
void showADC(void)//show output of ADC 1 + 2
{
	Temperature=ReadADC(1);
	ili9341_setcursor(10,20);
	printf("ADC1: %d", Temperature);
	Temperature=ReadADC(2);
	ili9341_setcursor(10,40);
	printf("ADC2: %d", Temperature);
	ili9341_setcursor(10,0);
		//push button
		if(JOY_PUSH && (!entprell))
		{
			printf("PUSH");
		}
		//horizontal
		if(JOY_HOR LEFT)
		{
			printf("LEFT");
		}else if(JOY_HOR RIGHT)
		{
			printf("RIGHT");
		}
		//vertical
		if(JOY_VERT UP)
		{
			printf("UP");
		}else if(JOY_VERT DOWN)
		{
			printf("DOWN");
		}
		if((JOY_HOR < 600) && (JOY_HOR>400) && (JOY_VERT<600) && (JOY_VERT>400) && !JOY_PUSH)
		{
			ili9341_setcursor(10,0);
			printf("     ");
		}
	
}

//SD-Card Functions
uint8_t read_line(char* buffer, uint8_t buffer_length)
{
    memset(buffer, 0, buffer_length);

    uint8_t read_length = 0;
    while(read_length < buffer_length - 1)
    {
        uint8_t c = uart_getc();

        if(c == 0x08 || c == 0x7f)
        {
            if(read_length < 1)
                continue;

            --read_length;
            buffer[read_length] = '\0';

            uart_putc(0x08);
            uart_putc(' ');
            uart_putc(0x08);

            continue;
        }

        uart_putc(c);

        if(c == '\n')
        {
            buffer[read_length] = '\0';
            break;
        }
        else
        {
            buffer[read_length] = c;
            ++read_length;
        }
    }

    return read_length;
}

uint32_t strtolong(const char* str)
{
    uint32_t l = 0;
    while(*str >= '0' && *str <= '9')
        l = l * 10 + (*str++ - '0');

    return l;
}

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
    {
        if(strcmp(dir_entry->long_name, name) == 0)
        {
            fat_reset_dir(dd);
            return 1;
        }
    }

    return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}

uint8_t print_disk_info(const struct fat_fs_struct* fs)
{
    if(!fs)
        return 0;

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info))
        return 0;

    uart_puts_p(PSTR("manuf:  0x")); uart_putc_hex(disk_info.manufacturer); uart_putc('\n');
    uart_puts_p(PSTR("oem:    ")); uart_puts((char*) disk_info.oem); uart_putc('\n');
    uart_puts_p(PSTR("prod:   ")); uart_puts((char*) disk_info.product); uart_putc('\n');
    uart_puts_p(PSTR("rev:    ")); uart_putc_hex(disk_info.revision); uart_putc('\n');
    uart_puts_p(PSTR("serial: 0x")); uart_putdw_hex(disk_info.serial); uart_putc('\n');
    uart_puts_p(PSTR("date:   ")); uart_putw_dec(disk_info.manufacturing_month); uart_putc('/');
                                   uart_putw_dec(disk_info.manufacturing_year); uart_putc('\n');
    uart_puts_p(PSTR("size:   ")); uart_putdw_dec(disk_info.capacity / 1024 / 1024); uart_puts_p(PSTR("MB\n"));
    uart_puts_p(PSTR("copy:   ")); uart_putw_dec(disk_info.flag_copy); uart_putc('\n');
    uart_puts_p(PSTR("wr.pr.: ")); uart_putw_dec(disk_info.flag_write_protect_temp); uart_putc('/');
                                   uart_putw_dec(disk_info.flag_write_protect); uart_putc('\n');
    uart_puts_p(PSTR("format: ")); uart_putw_dec(disk_info.format); uart_putc('\n');
    uart_puts_p(PSTR("free:   ")); uart_putdw_dec(fat_get_fs_free(fs)); uart_putc('/');
                                   uart_putdw_dec(fat_get_fs_size(fs)); uart_putc('\n');

    return 1;
}

#if FAT_DATETIME_SUPPORT
void get_datetime(uint16_t* year, uint8_t* month, uint8_t* day, uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    *year = 2007;
    *month = 1;
    *day = 1;
    *hour = 0;
    *min = 0;
    *sec = 0;
}
#endif
