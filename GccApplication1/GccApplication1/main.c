
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "glcd.h"
#include "gyro.h"
#include <math.h>
#include <avr/eeprom.h>

#define DHT11_PIN 6
#define LCD_Dir  DDRA
#define LCD_Port PORTA
#define RS PA1
#define EN PA2
#define sw1 PC0
#define sw2 PC1
#define sw3 PC2
#define sw4 PC3
#define sw5 PC4

#define l0 PE0
#define l1 PE1
#define l2 PE2
#define l3 PE3
#define l4 PE4
#define pwm PE5
#define buz PE6
#define servo PB6
#define fan PB7

int outs=0;
int dd=21;
unsigned int baudrate[15]={832, 416, 207, 138, 103, 68, 51, 34, 25, 16, 8, 7, 3, 1, 0};
unsigned char temp;
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;
char data[6];
char out[12];
char tem[5];
char hum[5];
int ttrig=0;
char err_code[4][13];
int tdiv=10;
int ltdiv=10;
int divtrig=0;
int autotrig=0;
int trig=0;
int bk=0;
int maxtem=50;
int mintem=10;
int maxhum=80;
int minhum=10;
int maxlux=99;
int minlux=0;
int sel=10;
int det=0;
int fcnt=0;
int fr=0;
int var=0;
int t=0;
char adc0[16]={0};
char adc1[16]={0};
char adc2[16]={0};
int h=0;
int m=0;
int s=0;
int hc=0;
int mc=0;
int sc=0;
unsigned char ot[9];
unsigned char ots[10][9];
unsigned char ote[10][9];
int rec=0;
char key=0;
int tint=0;
int fpwmon=0;
int fpwmtrig=0;
int spwmon=0;
int spwmtrig=0;
int z=0;
int zz=0;
int num=0;
int bsel[10]={0};
int bover[10]={0};
int etrig=0;
int adc=0;
int btg=0;
int test=0;
int lcnt=0;
int scon=1;
int fcon=1;
int gcon=1;

void timer0_init()
{
	TIMSK |= (1<<TOIE0) ;
	OCR0=0;
	TCCR0 |= (0 << CS02)|(1 << CS01)|(0 << CS00);
	TCNT0=0x88;
}
void timer2_init()
{
	TIMSK|=(1<<TOIE2);
	OCR2=0;
	TCCR2=(0<<WGM20) | (0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<CS22) | (1<<CS21) | (1<<CS20);
	TCNT2=0x06;
}

ISR(TIMER0_OVF_vect)
{
	TCNT0=0x88;
	if(spwmtrig==1)
	{
		zz++;
		if(zz<spwmon)PORTE|=(1<<pwm);
		if(zz>=spwmon)PORTE&=~(1<<pwm);
		if(zz>=400){zz=0;}
	}
}
ISR(TIMER2_OVF_vect)
{
	TCNT2=0x06;
	
	t++;
	if(t>=2000){t=0;}
	
	if(ttrig==1)
	{tint++;}
	
	if(tint>=1000)
	{
		sc++;
		if(sc==60){sc=0; mc++;}
		if(mc==60){mc=0; hc++;}
		if(hc==24){hc=0; mc=0; sc=0;}
		
		tint=0;
		
	}
	
	if(sel==1){lcnt++;}
	
	if(fpwmtrig==1)
	{
		z++;
		if(z<=fpwmon)PORTB|=(1<<fan);
		if(z>fpwmon)PORTB&=~(1<<fan);
		if(z>=20){z=0;}
	}
}

void serial1_init(void)
{
	DDRD |= 0x08;
	UBRR1H = baudrate[2]>>8;
	UBRR1L = baudrate[2];
	UCSR1A = 0x02;
	UCSR1B = 0x18;
	UCSR1C = 0x06;
	temp = UDR1;
}
unsigned char rx(void)
{
	while(1)
	{
		if((UCSR1A&0x80) != 0x00){return UDR1;}
		else return key;
	}
	
}
void serial_tx(unsigned char tx_data)
{
	while((UCSR1A&0x20) == 0x00);
	UDR1 = tx_data;
}
void print(unsigned char asd[50])
{
	int v=0;
	
	while(1)
	{
		serial_tx(asd[v]);
		v++;
		
		if(asd[v]=='\0'){break;}
	}
	_delay_us(1);
}

void Request()
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);
	_delay_ms(20);
	PORTD |= (1<<DHT11_PIN);
}
void Response()
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}
uint8_t Receive_data()
{
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))
		c = (c<<1)|(0x01);
		else
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

void adc_init()
{
	ADMUX=(1<<REFS0);
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
uint16_t read_adc(uint8_t ch)
{
	ch=ch&0b00000111;
	ADMUX|=ch;
	ADCSRA|=(1<<ADSC);
	while(!(ADCSRA & (1<<ADIF)));
	ADCSRA|=(1<<ADIF);
	return(ADC);
}

void loading()
{
	int i=0;
	while(1)
	{
		for(t=0; t<200;)
		{
			g_xy(16,8);
			prints("Word Skills 2018");
			
			g_xy(10,24);
			prints("Smart Farm Manager");
			
			g_xy(8,40);
			prints("Now Initializing...");
		}
		i++;
		
		g_f_rec(6,53,12*i,60);
		g_render();
		if(i==10) break;
	}
	
}
void setprint()
{
	g_clear();
	
	g_xy(60,0);
	prints("max");
	g_xy(96,0);
	prints("min");
	
	g_xy(0,16);
	prints("set_temp:");
	g_xy(63,16);
	printi(maxtem);
	g_xy(99,16);
	printi(mintem);
	
	g_xy(0,32);
	prints("set_humi:");
	g_xy(63,32);
	printi(maxhum);
	g_xy(99,32);
	printi(minhum);
	
	g_xy(0,48);
	prints("set_lux:");
	g_xy(63,48);
	printi(maxlux);
	g_xy(99,48);
	printi(minlux);
	
	g_render();
}
void setting()
{
	int maxy1=14;
	int maxy2=24;
	int x1=60;
	int x2=76;
	int trig=5;
	int bk=0;
	
	DDRC=0x00;

	setprint();
	PORTC=0x0f;
	g_rec(x1,maxy1,x2,maxy2);
	g_render();
	while(1)
	{
		
		if(!(PINC&=(1<<sw1))||!(PINC&=(1<<sw3)))
		{
			while(!(PINC&=(1<<sw1))||!(PINC&=(1<<sw3))){}
			if(x1==60){setprint(); x1=96; x2=112;}
			else{setprint(); x1-=36; x2-=36;}
			g_rec(x1,maxy1,x2,maxy2);
			g_render();
		}
		
		if(!(PINC&=(1<<sw2)))
		{
			while(!(PINC&=(1<<sw2))){}
			if(maxy1==14){setprint(); maxy1=46; maxy2=56;}
			else{setprint(); maxy1-=16; maxy2-=16;}
			g_rec(x1,maxy1,x2,maxy2);
			g_render();
		}
		
		if(!(PINC&=(1<<sw4)))
		{
			while(!(PINC&=(1<<sw4))){}
			if(maxy1==46){setprint(); maxy1=14; maxy2=24;}
			else{setprint(); maxy1+=16; maxy2+=16;}
			g_rec(x1,maxy1,x2,maxy2);
			g_render();
		}
		
		if(!(PINC&=(1<<sw5)))
		{
			trig=0;
			while(!(PINC&=(1<<sw5)))
			{
				for(t=0; t<1000;){if(PINC&=(1<<sw5)){break;}}
				trig++;
				if(trig==3){while(!(PINC&=(1<<sw5))){} bk=1; break;}
				
			}
			
		}
		
		
		if(bk==1){break;}
		
		if(trig<3)
		{
			trig=5;
			g_i_rec(x1,maxy1,x2,maxy2);
			g_render();
			while(1)
			{
				if(!(PINC&=(1<<sw2)))
				{
					while(!(PINC&=(1<<sw2))){}
					
					if(x1==60&&maxy1==14)
					{
						if(maxtem<50)
						{maxtem++; setprint(); g_i_rec(x1,maxy1,x2,maxy2); }
					}
					if(x1==96&&maxy1==14)
					{
						if(mintem<25)
						{mintem++; setprint(); g_i_rec(x1,maxy1,x2,maxy2); }
					}
					
					
					if(x1==60&&maxy1==30)
					{
						if(maxhum<95)
						{maxhum++; setprint(); g_i_rec(x1,maxy1,x2,maxy2); }
					}
					if(x1==96&&maxy1==30)
					{
						if(minhum<50)
						{minhum++; setprint(); g_i_rec(x1,maxy1,x2,maxy2); }
					}
					
					if(x1==60&&maxy1==46)
					{
						if(maxlux<99)
						{maxlux++; setprint(); g_i_rec(x1,maxy1,x2,maxy2); }
					}
					if(x1==96&&maxy1==46)
					{
						if(minlux<50)
						{minlux++; setprint(); g_i_rec(x1,maxy1,x2,maxy2);}
					}
					g_render();
				}
				
				
				if(!(PINC&=(1<<sw4)))
				{
					while(!(PINC&=(1<<sw4))){}
					
					if(x1==60&&maxy1==14)
					{
						if(maxtem>25)
						{maxtem--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
					if(x1==96&&maxy1==14)
					{
						if(mintem>0)
						{mintem--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
					
					
					if(x1==60&&maxy1==30)
					{
						if(maxhum>50)
						{maxhum--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
					if(x1==96&&maxy1==30)
					{
						if(minhum>0)
						{minhum--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
					
					if(x1==60&&maxy1==46)
					{
						if(maxlux>50)
						{maxlux--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
					if(x1==96&&maxy1==46)
					{
						if(minlux>0)
						{minlux--; setprint(); g_i_rec(x1,maxy1,x2,maxy2); g_render();}
					}
				}
				
				
				if(!(PINC&=(1<<sw5)))
				{
					while(!(PINC&=(1<<sw5))){}
					setprint();
					g_rec(x1,maxy1,x2,maxy2);
					g_render();
					break;
				}
				
			}
		}

		
		
		
	}
	
	
	
}
void onset()
{
	int cur=0;

	g_clear();
	while(1)
	{
		if(!(PINC&=(1<<sw1))&&cur>0)
		{
			while(!(PINC&=(1<<sw1))){}
			cur--;
		}
		
		if(!(PINC&=(1<<sw3))&&cur<2)
		{
			while(!(PINC&=(1<<sw3))){}
			cur++;
		}
		
		if(!(PINC&=(1<<sw2)))
		{
			while(!(PINC&=(1<<sw2))){}
			g_clear();
			while(1)
			{
				if(cur==0&&scon==0){scon=1;}
				if(cur==1&&fcon==0){fcon=1;}
				if(cur==2&&gcon==0){gcon=1;}
				
				if(scon==1||fcon==1||gcon==1){break;}
			}
		}
		
		if(!(PINC&=(1<<sw4)))
		{
			while(!(PINC&=(1<<sw4))){}
			g_clear();
			while(1)
			{
				if(cur==0&&scon==1){scon=0;}
				if(cur==1&&fcon==1){fcon=0;}
				if(cur==2&&gcon==1){gcon=0;}
				
				if(scon==0||fcon==0||gcon==0){break;}
			}
		}
		
		
		if(!(PINC&=(1<<sw5)))
		{
			while(!(PINC&=(1<<sw5))){}
			break;
		}
		
		switch(cur)
		{
			case 0:
			g_clear();
			g_xy(16,16);
			printc('$');
			break;
			
			case 1:
			g_clear();
			g_xy(58,16);
			printc('$');
			break;
			
			case 2:
			g_clear();
			g_xy(88,16);
			printc('$');
			break;
		}
		g_xy(10,0);
		prints("Power Setting Mode");
		
		g_xy(16,16);
		prints("Servo");
		g_xy(58,16);
		prints("Fan");
		g_xy(88,16);
		prints("Gyro");
		
		g_xy(5,32);
		prints("On");
		
		g_xy(2,48);
		prints("Off");
		
		g_rec(22,32,40,56);
		if(scon==1) g_f_rec(25,35,37,41);
		else g_f_rec(25,47,37,53);
		
		g_rec(57,32,75,56);
		if(fcon==1) g_f_rec(60,35,72,41);
		else g_f_rec(60,47,72,53);
		
		g_rec(91,32,109,56);
		if(gcon==1) g_f_rec(94,35,106,41);
		else g_f_rec(94,47,106,53);
		
		g_render();
		
		if(fcon==0) {DDRB&=~(1<<fan); PORTB&=~(1<<fan);}
		else {DDRB|=(1<<fan);}
		
		if(scon==0) {DDRB&=~(1<<servo); PORTB&=~(1<<servo);}
		else {DDRB|=(1<<servo);}
	}
}
void arrow()
{
	key=rx();
	if(key==27)
	{
		while(key==27){key=rx();}
		if(key==91)
		{
			while(key==91){key=rx();}
			if(key==65)
			{rec=1;}
			if(key==66)
			{rec=2;}
			if(key==68)
			{rec=3;}
			if(key==67)
			{rec=4;}
		}
	}
}
void timeset()
{
	int i=0;
	int sel=0;
	
	test=eeprom_read_byte((uint16_t*)46);
	while(1)
	{
		if(test==1)
		{
			ttrig=1;
			sc=eeprom_read_byte((uint16_t*)54);
			mc=eeprom_read_byte((uint16_t*)62);
			hc=eeprom_read_byte((uint16_t*)70);
			tint=100;
			break;
		}
		g_clear();
		g_xy(28,8);
		prints("Time Setting");
		
		sprintf(&ot[0],"%02d",hc);
		ot[2]=':';	sprintf(&ot[3],"%02d",mc);
		ot[5]=':';
		sprintf(&ot[6],"%02d",sc);
		g_xy(40,32);
		prints(ot);
		
		if(sel>=2)
		{g_xy(40+(6*sel)+6,32);}
		
		if(sel>=4)
		{g_xy(40+(6*sel)+12,32);}
		
		if(sel<2)
		{g_xy(40+(6*sel),32);}
		printc('$');
		
		for(i=0; i<3; i++)
		{
			g_xy(43+(18*i),24);
			printc('@');
			g_xy(43+(18*i),40);
			printc('#');
		}
		
		g_render();
		arrow();
		
		if(!(PINC&=(1<<sw1))){while(!(PINC&=(1<<sw1))){} rec=3;}
		if(!(PINC&=(1<<sw2))){while(!(PINC&=(1<<sw2))){} rec=1;}
		if(!(PINC&=(1<<sw3))){while(!(PINC&=(1<<sw3))){} rec=4;}
		if(!(PINC&=(1<<sw4))){while(!(PINC&=(1<<sw4))){} rec=2;}
		
		if(rec==3&&sel>0){sel--;}
		
		if(rec==4&&sel<5){sel++;}
		
		if(rec==1&&sel==0)
		{if(hc<14){hc+=10;}}
		if(rec==2&&sel==0)
		{if(hc>=10){hc-=10;}}
		
		if(rec==1&&sel==1)
		{if(hc<23){hc++;}}
		if(rec==2&&sel==1)
		{if(hc>0){hc--;}}
		
		if(rec==1&&sel==2)
		{if(mc<50){mc+=10;}}
		if(rec==2&&sel==2)
		{if(mc>0){mc-=10;}}
		
		if(rec==1&&sel==3)
		{if(mc<59){mc++;}}
		if(rec==2&&sel==3)
		{if(mc>0){mc--;}}
		
		if(rec==1&&sel==4)
		{if(sc<50){sc+=10;}}
		if(rec==2&&sel==4)
		{if(sc>0){sc-=10;}}
		
		if(rec==1&&sel==5)
		{if(sc<59){sc++;}}
		if(rec==2&&sel==5)
		{if(sc>0){sc--;}}
		
		rec=0;
		
		if(!(PINC&=(1<<sw5)))
		{
			while(!(PINC&=(1<<sw5))){}
			ttrig=1;
			eeprom_write_byte((uint16_t*)46,1);
			break;
		}
		
		if(key==13)
		{
			ttrig=1;
			eeprom_write_byte((uint16_t*)46,1);
			break;
		}
	}
}
void nowtime()
{
	eeprom_write_byte((uint16_t*)54,sc);
	eeprom_write_byte((uint16_t*)62,mc);
	eeprom_write_byte((uint16_t*)70,hc);
	
	s=eeprom_read_byte((uint16_t*)54);
	m=eeprom_read_byte((uint16_t*)62);
	h=eeprom_read_byte((uint16_t*)70);
	
	sprintf(&ot[0],"%02d",h);
	ot[2]=':';	sprintf(&ot[3],"%02d",m);
	ot[5]=':';
	sprintf(&ot[6],"%02d",s);
}
void error()
{
	int bnum=0;
	int br=0;
	
	print("\r\f\n                sw2:Err_Log\n\n\rsw1:T/H Graph   sw5:Time/div   sw3:L Graph\n\n\r                sw4:Vigil Mod");
	print("\n\n\r               -View Error Log-");
	
	while(1)
	{
		for(t=0; t<300;)
		{
			if(!(PINC&=(1<<sw2))&&bnum<num-1)
			{
				while(!(PINC&=(1<<sw2))){}
				t=300;
				bnum++;
			}
			
			if(!(PINC&=(1<<sw4))&&bnum>0)
			{
				while(!(PINC&=(1<<sw4))){}
				t=300;
				bnum--;
			}
			if(!(PINC&=(1<<sw5)))
			{
				while(!(PINC&=(1<<sw5))){}
				br=1; break;
			}
		}
		if(br==1){g_clear();
			g_line(10,0,10,63);
			g_line(0,53,127,53);
		g_render();  break;}
		g_clear();
		g_xy(61,0);
		printc('@');
		g_xy(11,8);
		prints("-View Error Log ");
		printi(bnum); printc('-');
		g_xy(25,24);
		if(bover[bnum]==1)
		{prints("Err_Over_Max");}
		else{prints("Err_Over_Min");}
		
		g_xy(28,32);
		prints("Sensor:");
		if(bsel[bnum]==0)
		{prints("Temp");}
		else{prints("Humi");}
		
		g_xy(7,40);
		prints("start_time:");
		prints(ots[bnum]);
		g_xy(13,48);
		prints("end_time:");
		prints(ote[bnum]);
		
		g_xy(61,56);
		printc('#');
		g_render();
	}
}
void divset()
{
	g_clear();
	g_render();
	while(1)
	{
		g_xy(16,8);
		prints("Time/div Setting");
		
		if(divtrig==0)
		{
			g_xy(22,24);
			prints("1.5s/ ");
			printi(tdiv);
		}
		
		if(divtrig==1)
		{
			g_xy(22,24);
			prints("0.1s/ ");
			printi(ltdiv);
		}
		
		prints(" Pixel");
		g_render();
		
		g_xy(7,48);
		prints("[sw5] Back to Graph");
		
		if(!(PINC&=(1<<sw2)))
		{
			while(!(PINC&=(1<<sw2))){}
			if(divtrig==0)
			{
				if(tdiv<10){tdiv++;}
				g_clearline(3);
				g_xy(22,24);
				prints("1.5s/ ");
				printi(tdiv);
				prints(" Pixel");
				g_render();
			}
			if(divtrig==1)
			{
				if(ltdiv<10){ltdiv++;}
				g_clearline(3);
				g_xy(22,24);
				prints("0.1s/ ");
				printi(ltdiv);
				prints(" Pixel");
				g_render();
			}
		}
		
		if(!(PINC&=(1<<sw4)))
		{
			while(!(PINC&=(1<<sw4))){}
			if(divtrig==0)
			{
				if(tdiv>5){tdiv--;}
				g_clearline(3);
				g_xy(22,24);
				prints("1.5s/ ");
				printi(tdiv);
				prints(" Pixel");
				g_render();
			}
			if(divtrig==1)
			{
				if(ltdiv>5){ltdiv--;}
				g_clearline(3);
				g_xy(22,24);
				prints("0.1s/ ");
				printi(ltdiv);
				prints(" Pixel");
				g_render();
			}
		}
		
		if(!(PINC&=(1<<sw5)))
		{
			while(!(PINC&=(1<<sw5)))
			t=0;
			g_clear();
			g_render();
			g_xy(34,24);
			prints("Now Saving");
			g_xy(24,32);
			prints("Please wait...");
			g_render();
			trig=1;
			break;
		}
		
	}
}
void war()
{
	g_clear();
	g_xy(37,16);
	prints("Intrusion");
	g_xy(34,24);
	prints("Detected!!!");
	
	while(1)
	{
		
		for(t=0; t<500;)
		{
			if(!(PINC&=(1<<sw5)))
			{
				btg=1;
				break;
			}
			g_xy(7,32);
			prints("-sw5- Back to graph");
			g_render();
			PORTE|=(1<<buz);
		}
		
		for(t=0; t<500;)
		{
			if(!(PINC&=(1<<sw5)))
			{
				btg=1;
				break;
			}
			g_xy(7,32);
			prints("-sw5- Back to graph");
			g_render();
			PORTE&=~(1<<buz);
		}
		if(btg==1)break;
	}
	PORTE&=~(1<<buz);
}
void vigil()
{
	char adca[16]={0};
	int adc=0;
	int adcmax=50;
	rec=0;
	while(1)
	{
		fr=0;
		for(t=0; t<100;)
		{
			if(!(PINF&=(1<<PF0))&&fcnt==0){fcnt=1;}
			if(PINF&=(1<<PF0)&&fcnt==1){fcnt=0; fr++;}
			
			arrow();
			if(rec==1&&adcmax<100)
			{
				adcmax+=5;
				rec=0;
			}
			if(rec==2&&adcmax>0)
			{
				adcmax-=5;
				rec=0;
			}
			
			
			if(!(PINC&=(1<<sw5)))
			{
				while(!(PINC&=(1<<sw5))){}
				btg=1;
			}
			
			if(btg==1)break;
		}
		if(btg==1)break;
		adc_init();
		adc=read_adc(1);
		adc=(adc-10)/2;
		if(adc<=0)adc=0;
		if(adc>=100)adc=100;
		sprintf(&adca[0],"%d",adc);
		
		g_clear();
		g_xy(22,0);
		prints("Vigilance Mode");

		g_xy(21,16);
		prints("Sound Value:");
		prints(adca);
		
		g_xy(22,32);
		prints("Light Value:");
		if(fr>=100)
		fr=100;
		
		if(fr==0)
		fr=1;
		printi(100-fr);
		if(maxlux<100-fr){war();}
		if(minlux>100-fr){war();}
		if(adcmax<adc){war();}
		g_xy(28,48);
		prints("Max_Sound:");
		printi(adcmax);
		g_render();
		
	}
}
void dht()
{
	int keyt=0;

	if(autotrig==0)
	{
		for(t=0; t<1500;)
		{
			
			if(!(PINC&=(1<<sw5)))
			{
				while(!(PINC&=(1<<sw5))){}
				divtrig=0;
				divset();
				
			}
			
			if(!(PINC&=(1<<sw3)))
			{
				while(!(PINC&=(1<<sw3))){}
				t=0;
				bk=1;
				break;
			}
			
			if(!(PINC&=(1<<sw2))&&etrig==1)
			{
				while(!(PINC&=(1<<sw2))){}
				error();
				t=0;
			}
			
			if(!(PINC&=(1<<sw4)))
			{
				while(!(PINC&=(1<<sw4))){}
				vigil();
				break;
			}
			
			key=rx();
			if(key==49){setting(); t=0; key=0; det=1; break;}
			if(key==50){onset(); t=0; key=0; det=1; break;}
			if(key==114){ keyt=1;}
			
			if(keyt==1&&key==51)
			{
				eeprom_write_byte((uint16_t*)46,0);
				hc=0;
				mc=0; sc=0;
				ttrig=0;
				timeset();
				t=0;
				key=0;
				keyt=0;
				det=1;
				break;
			}
		}
	}

	if(bk==0)
	{
		Request();
		Response();
		
		I_RH=Receive_data();
		D_RH=Receive_data();
		I_Temp=Receive_data();
		D_Temp=Receive_data();
		CheckSum=Receive_data();

		itoa(I_Temp,data,10);
		sprintf(&tem[0],"%s",data);
		
		itoa(D_Temp,data,10);
		sprintf(&tem[3],"%s",data);
		tem[2]='.';
		
		itoa(I_RH,data,10);
		sprintf(&hum[0],"%s",data);
		
		itoa(D_RH,data,10);
		sprintf(&hum[3],"%s",data);
		
		hum[2]='.';
	}
}
void pes()
{
	double x=0;
	double y=0;
	char asd[20];
	int p=0;
	double angle=0;
	int going=0;
	int br=0;

	spwmtrig=1;
	PORTB|=(1<<servo);
	I2C_Init();
	MPU6050_Init();
	autotrig=1;
	
	while(1)
	{
		
		if(gcon==1)
		{
			for(p=0; p<6; p++)
			{
				nowtime();
				
				print("\r\fPesticide Cooler Control...");
				print("\n\n\r          ");
				nowtime();
				print(ot);
				print("\n\n\rangle:");
				print(asd);
				going+=2;
				if(going>100){br=1; break;}
				sprintf(asd,"%d",going);
				print("        ");
				print(asd);
				print("%");
				
				for(t=0; t<250;)
				{
					read_gyro();
					
					x=(((Xa*100)+100)*0.0157);
					y=3.14-(((Ya*100)+100)*0.0157);

					spwmon=(y/0.1046)+10;
					if(spwmon>=39) spwmon=39;
					angle=y*57.324;
					if(angle<=0) {angle=0;}
					if(angle>=180) {angle=180;}
					
					sprintf(asd,"%3.1f",angle);
					
					g_clear();
					g_line(64,63,64-(40*cos(y)),63-(40*sin(y)));
					g_line(61,63,64-(40*cos(y)),63-(40*sin(y)));
					g_line(67,63,64-(40*cos(y)),63-(40*sin(y)));
					g_render();
					
				}
				
			}
		}
		if(br==1) break;
	}
	
	autotrig=0;
	spwmtrig=0;
}
void autorun()
{
	int cnt=0;
	int  i=0;
	int lc=0;
	
	etrig=1;
	//ttrig=1;
	nowtime();
	for(i=0; i<9; i++)
	{ots[num][i]=ot[i];}
	
	PORTE&=~(1<<buz);
	PORTE&=0x00;
	g_clear();
	g_xy(13,0);

	prints("Code:");
	if(sel==0){prints("Tem_Over_Max");}
	if(sel==1){prints("Tem_Over_Min");}
	if(sel==2){prints("Hum_Over_Max");}
	if(sel==3){prints("Hum_Over_Min");}
	g_render();
	autotrig=1;
	
	print("\r\f\n                sw2:Err_Log\n\n\rsw1:T/H Graph   sw5:Time/div   sw3:L Graph\n\n\r                sw4:Vigil Mod");
	print("\n\n\r               Error Detected!\n\r             Now Auto-Working....");
	
	while(1)
	{
		for(i=0; i<3; i++)
		{
			for(t=0; t<500;)
			{
				nowtime();
				g_clearline(2);
				g_xy(40,16);
				prints(ot);
				g_render();

				if(sel==1&&cnt==0)
				{if(lcnt>=1000){lcnt=0; lc++;}}
				
				if(sel==1&&cnt==1)
				{if(lcnt>=800){lcnt=0; lc++;}}
				
				if(sel==1&&cnt==2)
				{if(lcnt>=600){lcnt=0; lc++;}}
				
				if(sel==1&&cnt==3)
				{if(lcnt>=400){lcnt=0; lc++;}}
				
				if(sel==1&&cnt==4)
				{if(lcnt>=200){lcnt=0; lc++;}}
				
				if(sel==1)
				{
					PORTE&=0xe0;
					if(lc==1){PORTE|=0x01;}
					if(lc==2){PORTE|=0x02;}
					if(lc==3){PORTE|=0x04;}
					if(lc==4){PORTE|=0x08;}
					if(lc==5){PORTE|=0x10;}

					if(lc==6)lc=1;
				}
			}
		}
		
		dht();
		
		g_clearline(4);
		if(sel==0||sel==1){g_xy(46,32); prints(tem); printc('^'); printc('C');}
		if(sel==2||sel==3){g_xy(49,32); prints(hum); printc('%');}
		
		if(sel==0)
		{
			fpwmtrig=0;
			spwmtrig=1;
			PORTB|=(1<<servo);
			if(maxtem<=I_Temp)
			{
				if(I_Temp-maxtem<=50)spwmon=(6*(I_Temp-maxtem))+11;
				if(spwmon>=40){spwmon=40;}
				cnt=I_Temp-maxtem;
				
			}
			
			else
			{
				spwmon=0;
				spwmtrig=0;
				cnt=0;
				for(i=0; i<9; i++)
				{ote[num][i]=ot[i];}
				bover[num]=1;
				bsel[num]=0;
				num++;
				if(num==10)num=0;
				break;
			}
			
			g_clearline(7);
			g_xy(0,56);
			prints("Door Open:");
		}
		
		if(sel==1)
		{
			fpwmtrig=0;
			spwmtrig=0;
			if(mintem>I_Temp)
			{
					cnt=mintem-I_Temp-1;
					if(cnt>=4)cnt=4;
			}
			
			else
			{
				cnt=0;
				lc=0;
				for(i=0; i<9; i++)
				{ote[num][i]=ot[i];}
				bover[num]=0;
				bsel[num]=0;
				num++;
				if(num==10)num=0;
				break;	
			}

			g_clearline(7);
			g_xy(0,56);
			prints("Heat Level: 1 2 3 4 5");
			g_xy((cnt*12)+72,56);
			printc('$');
			
		}
		
		if(sel==2)
		{
			fpwmtrig=1;
			spwmtrig=0;
			if(maxhum<=I_RH)
			{
				fpwmon=(I_RH-maxhum+5);
				if(fpwmon>=20)fpwmon=20;
				cnt=(fpwmon-5)/3;
			}
			else
			{
				fpwmon=0;
				fpwmtrig=0;
				cnt=0;
				for(i=0; i<9; i++)
				{ote[num][i]=ot[i];}
				bover[num]=1;
				bsel[num]=1;
				num++;
				if(num==10)num=0;
				break;
			}
			
			g_clearline(7);
			g_xy(0,56);
			prints("Fan Speed:");
		}
		
		if(sel==3)
		{
			fpwmtrig=1;
			spwmtrig=0;
			if(minhum>=I_RH)
			{
				fpwmon=(minhum-I_RH+5);
				if(fpwmon>=20)fpwmon=20;
				cnt=(fpwmon-5)/3;
			}
			else
			{
				fpwmon=0;
				fpwmtrig=0;
				cnt=0;
				for(i=0; i<9; i++)
				{ote[num][i]=ot[i];}
				bover[num]=1;
				bsel[num]=1;
				num++;
				if(num==10)num=0;
				break;
			}
			g_clearline(7);
			g_xy(0,56);
			prints("Fan Speed:");
		}
		
		if(cnt>=5)cnt=5;
		
		if(sel!=1)
		{
			PORTE&=0x00;
			if(cnt>=1){PORTE|=0x01;}
			if(cnt>=2){PORTE|=0x03;}
			if(cnt>=3){PORTE|=0x07;}
			if(cnt>=4){PORTE|=0x0f;}
			if(cnt>=5){PORTE|=0x1f;}
			
			
			for(i=0; i<cnt; i++)
			{
				g_xy(63+(13*i),56);
				printc('<');
			}
			g_render();
		}
	}
	autotrig=0;
	fpwmtrig=0;
	spwmtrig=0;
	
	PORTB&=~(1<<fan);
	PORTB&=~(1<<servo);
	PORTE&=0x00;
	sel=10;
}
void overout()
{
	outs++;
	if(outs<=5)
	{
		g_xy(dd,56);
		printc('>');
		g_render();
		dd+=20;
	}
	if(outs==5){dd=21;}
	if(outs>=6&&outs<=10)
	{
		g_xy(dd,56);
		printc('<');
		g_render();
		dd+=20;
		PORTE|=(1<<buz);
		if(outs-5>=1){PORTE|=0x01;}
		if(outs-5>=2){PORTE|=0x03;}
		if(outs-5>=3){PORTE|=0x07;}
		if(outs-5>=4){PORTE|=0x0f;}
		if(outs-5>=5){PORTE|=0x1f;}
		
	}

	if(outs==7||outs==9)
	{
		PORTE&=~(1<<buz);
	}
	
	if(outs==11){PORTE&=~(1<<buz); autorun();}
}
int over(int c)
{
	int i=0;
	
	dd=21;
	outs=0;
	autotrig=1;
	g_clear();
	
	g_xy(13,8);
	prints("Error:");
	if(c==0||c==1){prints("Temperature");}
	if(c==2||c==3){prints("  Humudity ");}

	g_xy(13,24);
	prints("Code:");
	prints(err_code[c]);
	
	g_xy(4,40);
	prints("Require Proper Work.");
	g_render();
	
	while(1)
	{
		for(i=0; i<3; i++)
		{
			for(t=0; t<500;)
			{
				
				nowtime();
				if(c==0&&maxtem>I_Temp){sel=10;}
				if(c==1&&mintem<I_Temp){sel=10; }
				if(c==2&&maxhum>I_RH){sel=10; }
				if(c==3&&minhum<I_RH){sel=10; }
				
				g_xy(0,0);
				g_clearline(0);
				g_render();
				
			}
			overout();
			if(sel==10){g_clear(); g_render(); break;}
		}
		if(sel==10){g_clear(); g_render(); break;}
		dht();
		det=1;
	}
}
void detect()
{
	if(maxtem<=I_Temp){sel=0;}
	if(sel!=10){over(sel); PORTE=~(1<<buz);}
	if(mintem>I_Temp){sel=1;}
	if(sel!=10){over(sel); PORTE=~(1<<buz);}
	
	if(maxhum<=I_RH){sel=2;}
	if(sel!=10){over(sel); PORTE=~(1<<buz);}
	if(minhum>=I_RH){sel=3;}
	if(sel!=10){over(sel); PORTE=~(1<<buz);}

	PORTE&=0x00;
}
void thprint()
{
	print("\n\rT/H Graph");
	print("\n\rT/div: 1.5Sec/");
	sprintf(out,"%d",tdiv);
	print(out);
	print("Pixel");
	print("\n\rmaxtem=");
	sprintf(out,"%d",maxtem);
	print(out);
	print("\n\rmintem=");
	sprintf(out,"%d",mintem);
	print(out);
	print("\n\rmaxhum=");
	sprintf(out,"%d",maxhum);
	print(out);
	print("\n\rminhum=");
	sprintf(out,"%d",minhum);
	print(out);
}
void lprint()
{
	print("\n\rL Graph");
	print("\n\rT/div: 0.1Sec/");
	sprintf(out,"%d",ltdiv);
	print(out);
	print("Pixel");
	print("\n\rmaxlux=");
	sprintf(out,"%d",maxlux);
	print(out);
	print("\n\rminlux=");
	sprintf(out,"%d",minlux);
	print(out);

}
void lgraph()
{
	int x0=10;
	int y0=0;
	int x1=10;
	int y1=0;
	int lpcnt=0;
	int keyt=0;
	
	divtrig=1;
	g_clear();
	g_line(10,0,10,63);
	g_line(0,53,127,53);
	g_render();
	
	print("\r\f\n1:Max/Min Setting\n\r2:Power Setting\n\rR+3:Time Reset and Re-setting time\n\r");
	lprint();
	
	while(1)
	{
		for(t=0; t<100;)
		{
			
			if(!(PINF&=(1<<PF0))&&fcnt==0){fcnt=1;}
			if(PINF&=(1<<PF0)&&fcnt==1){fcnt=0; fr++;}
			if(!(PINC&=(1<<sw5))){while(!(PINC&=(1<<sw5))){} divset(); _delay_ms(1500); t=100;}
			
			if(!(PINC&=(1<<sw3)))
			{
				while(!(PINC&=(1<<sw3)))
				bk=1;
				t=100;
			}

			if(!(PINC&=(1<<sw4)))
			{
				while(!(PINC&=(1<<sw4))){}
				vigil();
				g_clear();
				x0=10;
				btg=0;
				
				break;
			}
			
			key=rx();
			if(key==49)
			{
				setting();
				g_clear();
				g_line(10,0,10,63);
				g_line(0,53,127,53);
				g_render();
				t=0;
				key=0;
				det=1;
				break;
			}
			if(key==50)
			{
				onset();
				g_clear();
				g_line(10,0,10,63);
				g_line(0,53,127,53);
				g_render();
				t=0;
				key=0;
				det=1;
				break;
			}
			if(key==114) keyt=1;
			
			if(keyt==1&&key==51)
			{
				eeprom_write_byte((uint16_t*)46,0);
				hc=0;
				mc=0; sc=0;
				ttrig=0;
				timeset();
				t=0;
				key=0;
				keyt=0;
				trig=1;
				g_clear();
				g_line(10,0,10,63);
				g_line(0,53,127,53);
				g_render();
				break;
			}
			
		}
		
		lpcnt++;
		if(lpcnt==10)
		{
			lpcnt=0;
			print("\r\f\n1:Max/Min Setting\n\r2:Power Setting\n\rR+3:Time Reset and Re-setting time\n\r");
			lprint();
		}
		
		if(bk==1)
		{
			bk=0;
			break;
		}
		
		if(fr>=100)
		fr=100;
		
		if(fr==0)
		fr=1;
		
		if(x0==10)
		{
			y0=53-(100-fr)/2;
			x1=10;
			y1=y0;
		}
		
		if(x0>=127||trig==1)
		{
			g_clear();
			x0=10;
			x1=x0;
			trig=0;
		}
		
		g_clearline(7);
		
		g_line(10,0,10,63);
		g_line(0,53,127,53);
		
		g_xy(12,56);
		prints("lux:");
		printi(100-fr);
		printc('%');
		
		nowtime();
		g_xy(66,56);
		nowtime();
		prints(ot);
		
		g_cir(x1,y1,1);
		
		g_line(x0,y0,x1,y1);
		
		g_render();
		x1+=ltdiv;
		y1=53-(100-fr)/2;
		x0+=ltdiv;
		y0=54;
		
		fr=0;

	}
	
}
void thgraph()
{
	char out[10];
	int y0=0;
	int x0=10;
	int y1=0;
	int x1=0;
	
	int hy0=0;
	int hx0=10;
	int hy1=0;
	int hx1=0;
	
	divtrig=0;
	autotrig=0;
	g_clear();
	g_line(10,0,10,63);
	g_line(0,53,127,53);
	g_render();
	
	print("\r\f\n1:Max/Min Setting\n\r2:Power Setting\n\rR+3:Time Reset and Re-setting time\n\r");
	thprint();
	
	while(1)
	{
		dht();
		nowtime();
		print("\r\f\n1:Max/Min Setting\n\r2:Power Setting\n\rR+3:Time Reset and Re-setting time\n\r");
		thprint();
		if(bk==1)
		{
			bk=0;
			break;
		}
		
		detect();
		
		if(x0==10)
		{
			x1=10;
			hx1=10;
			y0=53-I_Temp;
			hy0=53-(I_RH/2);
		}
		
		if(x0!=10 && x0!=1)
		{
			y0=53-I_Temp;
			x0+=tdiv;
			
			hy0=53-(I_RH/2);
			hx0+=tdiv;
		}
		
		if(x0==1)
		{
			x0=10+tdiv;
			hx0=10+tdiv;
			
			y0=53-I_Temp;
			hy0=53-(I_RH/2);
		}
		
		
		
		if(x0>=127||trig==1||det==1||btg==1)
		{
			trig=0;
			x0=10;
			hx0=10;
			x1=10;
			hx1=10;
			g_clear();
			g_line(10,0,10,63);
			g_line(0,53,127,53);
			
			g_xy(12,56);
			prints("tem:");
			prints(tem);
			
			g_xy(70,56);
			prints("hum:");
			prints(hum);
			
			g_render();
			det=0;
			autotrig=0;
			btg=0;
		}
		
		
		
		g_clearline(7);
		g_line(10,0,10,63);
		g_line(0,53,127,53);
		
		g_xy(12,56);
		prints("tem:");
		prints(tem);
		
		g_xy(70,56);
		prints("hum:");
		prints(hum);

		
		g_line(x0,y0,x1,y1);
		g_cir(x0,y0,1);
		
		g_line(hx0,hy0,hx1,hy1);
		g_cir(hx0,hy0,1);
		
		g_render();
		
		if(x0!=10)
		{
			x1=x0;
			y1=y0;
			hx1=hx0;
			hy1=hy0;
		}
		
		if(x0==10)
		{
			x1=x0;
			y1=y0;
			hx1=hx0;
			hy1=hy0;
			
			x0=1;
			hx0=1;
		}
		

	}
}
int swtrig=0;
int otrig=0;
void mainmod()
{
	
	if(otrig==0)
	{
		g_clear();
		g_xy(31,0);
		prints("Select Mod:");
		g_xy(0,24);
		prints("sw1:Setting Mode");
		g_xy(0,40);
		prints("sw3:Operation Mode");
		g_render();
		
		test=eeprom_read_byte((uint16_t*)46);
		if(test==1) ttrig=1;
	}
	if(otrig==1)
	{
		g_clear();
		g_xy(22,0);
		prints("Operation Mode");
		g_xy(12,16);
		prints("Temp/Humi Graph");
		g_xy(12,32);
		prints("Light Graph");
		g_xy(12,48);
		prints("Pesticide Mode");
		g_xy(0,(swtrig*16)+16);
		printc('<');
		g_render();
		
		if(!(PINC&=(1<<sw4))&&swtrig<2)
		{
			while(!(PINC&=(1<<sw4))){}
			swtrig++;
		}
		if(!(PINC&=(1<<sw2))&&swtrig>0)
		{
			while(!(PINC&=(1<<sw2))){}
			swtrig--;
		}
		
		if(!(PINC&=(1<<sw5))&&swtrig==0)
		{
			while(!(PINC&=(1<<sw5))){}
			thgraph();	otrig=0;
			print("\r\f");
			print("loading...");
			print("      ok");
			print("\r\nvalue setting...");
			print("ok");
			print("\r\ntime setting...");
			print(" ok");
			print("\r\nPower setting...");
			print("ok");
		}
		if(!(PINC&=(1<<sw5))&&swtrig==1)
		{
			while(!(PINC&=(1<<sw5))){}
			lgraph(); otrig=0;
			print("\r\f");
			print("loading...");
			print("      ok");
			print("\r\nvalue setting...");
			print("ok");
			print("\r\ntime setting...");
			print(" ok");
			print("\r\nPower setting...");
			print("ok");
		}
		if(!(PINC&=(1<<sw5))&&swtrig==2)
		{
			while(!(PINC&=(1<<sw5))){}
			pes(); otrig=0;
			print("\r\f");
			print("loading...");
			print("      ok");
			print("\r\nvalue setting...");
			print("ok");
			print("\r\ntime setting...");
			print(" ok");
			print("\r\nPower setting...");
			print("ok");
		}
		
		
		
	}
	
	if(!(PINC&=(1<<sw1))&&otrig==0)
	{
		while(!(PINC&=(1<<sw1))){}
		
		print("\r\f");
		print("loading...");
		print("      ok");
		print("\r\nvalue setting...");
		setting();
		print("ok");
		print("\r\ntime setting...");
		timeset();
		print(" ok");
		print("\r\nPower setting...");
		onset();
		print("ok");
		
	}

	if(!(PINC&=(1<<sw3))&&otrig==0)
	{
		while(!(PINC&=(1<<sw3))){};
		otrig=1;
	}
}


int pos_x = 63;
int pos_y = 31;
int joy = 0;
int snake_length = 1;

int main(void)
{
	DDRB=0xff;
	DDRC=0x00;
	DDRE=0xff;
	PORTE&=~(1<<buz);
	
	sprintf(&err_code[0][0],"%s","Tem_Over_Max");
	sprintf(&err_code[1][0],"%s","Tem_Over_Min");
	sprintf(&err_code[2][0],"%s","Hum_Over_Max");
	sprintf(&err_code[3][0],"%s","Hum_Over_Min");
	
	sei();
	timer0_init();
	timer2_init();
	serial1_init();
	
	g_init();
	g_clear();
	g_font(Font5x8,5,8);
	PORTE&=0x00;
	g_render();
	
	autotrig=0;
	fpwmtrig=0;
	spwmtrig=0;
	
// 	test=eeprom_read_byte((uint16_t*)46);
// 	if(test==1)
// 	{
// 		sc=eeprom_read_byte((uint16_t*)54);
// 		mc=eeprom_read_byte((uint16_t*)62);
// 		hc=eeprom_read_byte((uint16_t*)70);
// 	}
	
	
	PORTB&=~(1<<fan);
	PORTB&=~(1<<servo);
	PORTE&=0x00;
	
	I2C_Init();
	MPU6050_Init();
	
	while (1)
	{
		gyro();
		
		switch(joy){
			case 1:	if(pos_y+1<63)	pos_y++;	if(pos_y+1<63)	pos_y++;	if(pos_y+1<63)	pos_y++;	break;
			case 2:	if(pos_y-1>0)	pos_y--;	if(pos_y-1>0)	pos_y--;	if(pos_y-1>0)	pos_y--;	break;
			case 3:	if(pos_x-1>0)	pos_x--;	if(pos_x-1>0)	pos_x--;	if(pos_x-1>0)	pos_x--;	break;
			case 4:	if(pos_x+1<127)	pos_x++;	if(pos_x+1<127)	pos_x++;	if(pos_x+1<127)	pos_x++;	break;
		}
		GLCD_SetPixels(pos_x-1,pos_y-1,pos_x+1,pos_y+1,w);
		g_render();
		_delay_ms(400);
		GLCD_SetPixels(pos_x-1,pos_y-1,pos_x+1,pos_y+1,b);
	}
}
// 		g_xy(16,8);
// 		printd(Xa,100);
// 		g_xy(16,16);
// 		printd(Ya,100);
// 		g_xy(16,24);
// 		printd(Za,100);
// 		g_render();


gyro(){
	
	read_gyro();
	if(Ya>1)  Ya =  1;
	if(Ya<-1) Ya = -1;
	if(Za>1)  Za =  1;
	if(Za<-1) Za = -1;
	int Y = (int)(Ya*100);
	int Z = (int)(Za*100);
	if(abs(Y)>=abs(Z)){
		if(Y>=0) joy = 3;
		else     joy = 4;
	}
	else{
		if(Z>=0) joy = 1;
		else     joy = 2;
	}
	//1위     z -1
	//2아래   z  1
	//3왼쪽   y  1
	//4오른쪽 y -1
}