/*
 Name:		ServoControl.ino
 Created:	4/30/2019 3:59:50 PM
 Author:	Rob Antonisse

 gebruikte deKOder heeft verschillende aanpassingen
 minder buffers



*/

#include <EEPROM.h>

//declarations common
byte COM_reg;

byte COM_mode;

//Declaraties deKoder
volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
boolean DEK_Monitor = false; //shows DCC commands as bytes
byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[6]; //registerbyte for 12 command buffers //bit7= free (false) bit0= CV(true)
byte DEK_Buf0[6];
byte DEK_Buf1[6];
byte DEK_Buf2[6];
byte DEK_Buf3[6];
byte DEK_Buf4[6];
byte DEK_Buf5[6];

//declarations shiftregisters
//byte SH_byte[2]; //2 bytes to be shifted out
//used GPIOR0 GPIOR1 and GPIOR2

byte SH_bitcount;
byte SH_bytecount;
byte SH_fase;

//declarations servo
byte servo; //adresses current handled servo in SER_exe
unsigned int SER_l[8]; //puls width left x10 in us   16000 = minimum?
unsigned int SER_r[8]; //puls width right 32000=maximum? 24000 = centre?
unsigned int SER_position[8]; //current position
//unsigned int SER_goal[8]; //goal, position to reach
byte SER_count[8];
unsigned int SER_speed[8];//motion speed of servo
byte SER_reg[8]; //register byte
byte SER_last; //holds last controlled servo (not DCC controlled)
byte SER_swm; // EEPROM 100; switch mode for the servo mono (default) true, dual false
byte SW_last[8];
byte COM_maxservo = 3;
unsigned int COM_dccadres = 1; //basic adres DCC receive
//unsigned long CLK_time;

byte LED_mode;
unsigned long LED_time;
int LED_count[2]; //two counters for led effects

//declaration for testing can be removed (later)
volatile unsigned long tijdmeting;
volatile unsigned long gemeten;

void setup() {
	//Serial.begin(9600);
	//shiftregisters init	
	DDRB |= (1 << 0); //pin 8 serial data out, set as output
	DDRB |= (1 << 1); //pin 9 Rclock, latch set as output
	DDRB |= (1 << 2); //pin 10 shift clock set as output	
	DDRB |= (1 << 3); //pin 11 OE (output enabled) van de shifts
	DDRB |= (1 << 4); //pin 12 red control led
	DDRB |= (1 << 5); //pin 13 green control led

	PORTB &= ~(1 << 4);
	PORTB |= (1 << 5);



	/*

	DDRC &= ~(1 << 0); //PIN A0 input switch kolom 1
	DDRC &= ~(1 << 1); //PIN A1 input switch kolom 2
	DDRC &= ~(1 << 2); //PIN A2 input switch kolom 3
	DDRC &= ~(1 << 3); //PIN A3 input switch kolom 4

	PORTC |= (1 << 0); //pullup resistor pin A0
	PORTC |= (1 << 1); //pullup resistor pin A1
	PORTC |= (1 << 2); //pullup resistor pin A2
	PORTC |= (1 << 3); //pullup resistor pin A3
*/

	PORTC = 0x0F;
	//DeKoder part, interrupt on PIN2

	DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//bitSet(EIMSK, INT0);//EIMSK – External Interrupt Mask Register bit0 INT0 > 1

	 //timer interupt tbv servo control
	TCCR1A = 0;
	//TCCR2B = 5; // |= (1 << 0); //set clock no prescaler 128
	//125 counter=1ms  250 counter=2ms 187 counter =1,5ms centre
	//TCNT2 – Timer / Counter Register
	//OCR2A = 187; // – Output Compare Register A
	//TIMSK2   Bit 1 – OCIE2A : Timer / Counter2 Output Compare Match A Interrupt Enable
	TIMSK1 |= (1 << 1); //enable interupt
	//servo part init
	//servo = 0;
	//test instellingen kan later eruit...
	MEM_init();
	//temp
	SER_init();
}

void MEM_init() {
	//reads and sets initial value from eeprom
	SER_swm = EEPROM.read(100); //switch modes for servo's

}
void MEM_change() {
	//checks for changes in Memorie bytes
	if (EEPROM.read(100) != SER_swm)EEPROM.write(100, SER_swm); //switch modes

}
ISR(INT0_vect) { //syntax voor een ISR
//isr van PIN2
	//DEK_Reg fase van bit ontvangst
	//bit 0= bitpart ok (1) of failed(0)
	//bit1= truepart 
	//bit2=falsepart
	//bit3= received bit true =true false =false
	//bit4=restart, begin, failed as true
	cli();
	DEK_duur = (micros() - DEK_Tperiode);
	DEK_Tperiode = micros();
	if (DEK_duur > 50) {//50  ****************
		if (DEK_duur < 62) { //62 **************
			DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);

			if (bitRead(DEK_Reg, 1) == false) {
				DEK_Reg &= ~(1 << 2); //bitClear(DekReg, 2);
				DEK_Reg |= (1 << 1);
			}
			else { //received full true bit
				DEK_Reg |= (1 << 3);
				DEK_BitRX();
				DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
				DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
			}
		}
		else {
			if (DEK_duur > 106) { //106 ***********************

				if (DEK_duur < 124) { //124 ***********preferred 118 6us extra space in false bit
					DEK_Reg |= (1 << 0); //bitSet(DekReg, 0);
					if (bitRead(DEK_Reg, 2) == false) {
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
						DEK_Reg |= (1 << 2);  //bitSet(DekReg, 2);
					}
					else { //received full false bit
						DEK_Reg &= ~(1 << 3); //bitClear(DekReg, 3);
						DEK_BitRX();
						DEK_Reg &= ~(1 << 2);//bitClear(DekReg, 2);
						DEK_Reg &= ~(1 << 1); //bitClear(DekReg, 1);
					}
				}
			}
		}
	}
	sei();
}
ISR(TIMER1_COMPA_vect) {
	SER_stop();
}
void DEK_begin() {//runs when bit is corrupted, or command not correct
	//lesscount++;
	DEK_countPA = 0;
	DEK_Reg = 0;
	DEK_Status = 0;
	for (int i = 0; i < 6; i++) {
		DEK_byteRX[i] = 0; //reset receive array
	}
}
void DEK_BufCom(boolean CV) { //create command in Buffer
	byte i = 0;
	while (i < 6) {

		if (bitRead(DEK_BufReg[i], 7) == false) {
			DEK_BufReg[i] = 0; //clear found buffer


			DEK_Buf0[i] = DEK_byteRX[0];
			DEK_Buf1[i] = DEK_byteRX[1];
			DEK_Buf2[i] = DEK_byteRX[2];

			if (CV == true) {
				DEK_BufReg[i] |= (1 << 0); //set for CV
				DEK_Buf3[i] = DEK_byteRX[3];
				DEK_Buf4[i] = DEK_byteRX[4];
				DEK_Buf5[i] = DEK_byteRX[5];
			}
			else {

				DEK_Buf3[i] = 0;
				DEK_Buf4[i] = 0;
				DEK_Buf5[i] = 0;
			}
			DEK_BufReg[i] |= (1 << 7); //claim buffer
			i = 15;
		}
		i++;
	} //close for loop
} //close void
void DEK_BitRX() { //new version
	static byte countbit = 0; //counter received bits
	static byte countbyte = 0;
	static byte n = 0;

	static byte check;

	DEK_Reg |= (1 << 4);//resets and starts process if not reset in this void
	switch (DEK_Status) {
		//*****************************
	case 0: //Waiting for preample 
		if (bitRead(DEK_Reg, 3) == true) {
			DEK_countPA++;
			if (DEK_countPA > 12) {
				DEK_Status = 1;
				countbit = 0;
				countbyte = 0;
			}
			bitClear(DEK_Reg, 4);
		}
		break;
		//*************************
	case 1: //Waiting for false startbit
		if (bitRead(DEK_Reg, 3) == false) { //startbit receive
			DEK_countPA = 0;
			DEK_Status = 2;
		}
		//if Dekreg bit 3= true no action needed.
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 2: //receiving data
		if (bitRead(DEK_Reg, 3) == true) DEK_byteRX[countbyte] |= (1 << (7 - countbit));
		countbit++;
		if (countbit == 8) {
			countbit = 0;
			DEK_Status = 3;
			countbyte++;
		}
		bitClear(DEK_Reg, 4); //correct, so resume process
		break;
		//*************************
	case 3: //waiting for separating or end bit
		if (bitRead(DEK_Reg, 3) == false) { //false bit
			DEK_Status = 2; //next byte
			if ((bitRead(DEK_byteRX[0], 6) == false) & (bitRead(DEK_byteRX[0], 7) == true))bitClear(DEK_Reg, 4); //correct, so resume process	
		}
		else { //true bit, end bit, only 3 byte and 6 byte commands handled by this dekoder
			switch (countbyte) {
			case 3: //Basic Accessory Decoder Packet received
				//check error byte
				if (DEK_byteRX[2] == (DEK_byteRX[0] ^ DEK_byteRX[1])) {
					//aanpassingen mei 2019 voor filter eenzelfde commandoos
					if (check != DEK_byteRX[2])DEK_BufCom(false);
					check = DEK_byteRX[2];
					//Serial.println(check);
				}

				break; //6
			case 6: ///Accessory decoder configuration variable Access Instruction received (CV)
				//in case of CV, handle only write command
				if (bitRead(DEK_byteRX[2], 3) == true && (bitRead(DEK_byteRX[2], 2) == true)) {
					//check errorbyte and make command
					if (DEK_byteRX[5] == DEK_byteRX[0] ^ DEK_byteRX[1] ^ DEK_byteRX[2] ^ DEK_byteRX[3] ^ DEK_byteRX[4])DEK_BufCom(true);
					//}
				}
				break;
			} //close switch bytecount
		}//close bittype
		break;
		//***************************************
	} //switch dekstatus
	if (bitRead(DEK_Reg, 4) == true)DEK_begin();
}
void DEK_DCCh() { //handles incoming DCC commands, called from loop()
	static byte n = 0; //one buffer each passing
	byte temp;
	int decoder;
	int channel = 1;
	int adres;
	boolean port = false;
	boolean onoff = false;
	int cv;
	int value;

	//translate command
	if (bitRead(DEK_BufReg[n], 7) == true) {
		decoder = DEK_Buf0[n] - 128;
		if (bitRead(DEK_Buf1[n], 6) == false)decoder = decoder + 256;
		if (bitRead(DEK_Buf1[n], 5) == false)decoder = decoder + 128;
		if (bitRead(DEK_Buf1[n], 4) == false)decoder = decoder + 64;
		//channel
		if (bitRead(DEK_Buf1[n], 1) == true) channel = channel + 1;
		if (bitRead(DEK_Buf1[n], 2) == true) channel = channel + 2;
		//port
		if (bitRead(DEK_Buf1[n], 0) == true)port = true;
		//onoff
		if (bitRead(DEK_Buf1[n], 3) == true)onoff = true;
		//CV
		if (bitRead(DEK_BufReg[n], 0) == true) {
			cv = DEK_Buf3[n];
			if (bitRead(DEK_Buf2[n], 0) == true)cv = cv + 256;
			if (bitRead(DEK_Buf2[n], 1) == true)cv = cv + 512;
			cv++;
			value = DEK_Buf4[n];
		}
		else {
			cv = 0;
			value = 0;
		}
		COM_exe(bitRead(DEK_BufReg[n], 0), decoder, channel, port, onoff, cv, value);
		//Show Monitor (bytes)
		if (DEK_Monitor == true) {
			Serial.print("buffer= ");
			Serial.print(n);
			Serial.print("  value:  ");
			Serial.print(bitRead(DEK_BufReg[n], 7));
			Serial.print(bitRead(DEK_BufReg[n], 6));
			Serial.print(bitRead(DEK_BufReg[n], 5));
			Serial.print(bitRead(DEK_BufReg[n], 4));
			Serial.print(bitRead(DEK_BufReg[n], 3));
			Serial.print(bitRead(DEK_BufReg[n], 2));
			Serial.print(bitRead(DEK_BufReg[n], 1));
			Serial.print(bitRead(DEK_BufReg[n], 0));
			Serial.println("");

			temp = DEK_Buf0[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf1[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf2[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf3[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");

			temp = DEK_Buf4[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");


			temp = DEK_Buf5[n];
			Serial.print(bitRead(temp, 7));
			Serial.print(bitRead(temp, 6));
			Serial.print(bitRead(temp, 5));
			Serial.print(bitRead(temp, 4));
			Serial.print(bitRead(temp, 3));
			Serial.print(bitRead(temp, 2));
			Serial.print(bitRead(temp, 1));
			Serial.print(bitRead(temp, 0));
			Serial.println("");
			Serial.println("------");

		}
		//clear buffer
		DEK_BufReg[n] = 0;
		DEK_Buf0[n] = 0;
		DEK_Buf1[n] = 0;
		DEK_Buf2[n] = 0;
		DEK_Buf3[n] = 0;
		DEK_Buf4[n] = 0;
		DEK_Buf5[n] = 0;
	}
	n++;
	if (n > 5)n = 0;
}
void COM_exe(boolean type, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//type=CV(true) or switch(false)
	//decoder basic adres of decoder 
	//channel assigned one of the 4 channels of the decoder (1-4)
	//Port which port R or L
	//onoff bit3 port on or port off
	//cv cvnumber
	//cv value
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//Applications 
	APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
	APP_function(type, adres, decoder, channel, port, onoff, cv, value);
}
void APP_Monitor(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	//application for DCC monitor
	if (type == true) {
		Serial.print("CV:   , ");
	}
	else {
		Serial.print("Switch, ");
	}
	Serial.print("Adres: ");
	Serial.print(adres);
	Serial.print("(");
	Serial.print(decoder);
	Serial.print("-");
	Serial.print(channel);
	Serial.print("), ");
	//cv
	if (type == true) {
		Serial.print("CV: ");
		Serial.print(cv);
		Serial.print(", waarde: ");
		Serial.print(value);
	}
	else {
		if (port == true) {
			Serial.print("A<, ");
		}
		else {
			Serial.print("R>, ");
		}
		if (onoff == true) {
			Serial.print("On.");
		}
		else {
			Serial.print("Off");
		}
	}
	Serial.println("");
}
void APP_function(boolean type, int adres, int decoder, int channel, boolean port, boolean onoff, int cv, int value) {
	byte dcc;
	//voor hoge dcc adressen gaat dit denk ik  niet goed later nog en naar kijken
	dcc = adres - COM_dccadres;
	//Serial.println(dcc);
	if (dcc < 8) { //0~7
		if (type == false) { //switch command
			SER_start(dcc, port);
		}
		else { //CV command
		}
	}
}
void SER_init() {
	//runs ones called from setup
	for (byte i = 0; i < 8; i++) {
		SER_l[i] = 18000; //left positio
		SER_r[i] = 30000; //right position
		SER_position[i] = 24000; //current position, initial start
		SER_speed[i] = 500; // 500;
	}
}
void SER_start(byte sv, boolean direction) {
	if (bitRead(SER_reg[sv], 0) == false) { //servo not running
		SER_reg[sv] |= (1 << 1); //set request for start
	}
	if (direction == true) {
		SER_reg[sv] |= (1 << 2);
	}
	else {
		SER_reg[sv] &= ~(1 << 2);
	}
}
void SER_stop() { //called from ISR//
	//PINB |= (1 << 4);
	GPIOR0 &= ~(1 << servo); //reset puls servo
	TCCR1B = 0; //stop timer 2	
	SHIFT();
	SER_run();
}
void SER_run() {
	byte count;
	byte temp;
	COM_reg ^= (1 << 0);
	TCNT1 = 0;
	if (bitRead(COM_reg, 0) == true) { //start servo	
		servo++;
		if (servo > 7) servo = 0;

		GPIOR0 &= ~(1 << servo); //clear bit
		if (bitRead(SER_reg[servo], 0) == true) {
			GPIOR0 |= (1 << servo); //set servo puls				
			SHIFT();
		}
		OCR1A = SER_position[servo]; // – set new step position in Output Compare Register A
		TCCR1B = 1; // |= (1 << 0); //set clock no prescaler	

		//uit ser_set gaat sneller, kan straks hier nu ff nog niet
		//Servo not active check 
		//SER_set();

	}
	else { //pauze interval between servo controlpulses
		SER_set(); //stond eerst hierboven
		OCR1A = 18000;// 18000; //18000
		TCCR1B = 2; //2 gives period of 6ms			
	}
}
void SER_set() { //called from SER_run 	
	if (bitRead(SER_reg[servo], 0) == true) { //servo runs
		switch (bitRead(SER_reg[servo], 2)) {
		case false:
			if (SER_position[servo] == SER_l[servo]) {
				SER_count[servo]++;
				if (SER_count[servo] > 4) {
					SER_reg[servo] &= ~(1 << 0); //stop servo
					SER_count[servo] = 0;
				}
			}
			else {
				SER_position[servo] = SER_position[servo] - SER_speed[servo]; //set new position
				if (SER_position[servo] < SER_l[servo]) { //postion reached
					SER_position[servo] = SER_l[servo];
				}
			}
			break;

		case true:
			if (SER_position[servo] == SER_r[servo]) {
				SER_count[servo]++;
				if (SER_count[servo] > 4) {
					SER_reg[servo] &= ~(1 << 0); //stop servo
					SER_count[servo] = 0;
				}

			}
			else {
				SER_position[servo] = SER_position[servo] + SER_speed[servo]; //set new position
				if (SER_position[servo] > SER_r[servo]) { //postion reached
					SER_position[servo] = SER_r[servo];
				}
			}
			break;
		}
	}
	else { //servo not active
		if (bitRead(SER_reg[servo], 1) == true) {
			//register settings
			SER_reg[servo] |= (1 << 0);
			SER_reg[servo] &= ~(1 << 1);
		}
	}
	GPIOR1 = (GPIOR1 << 1);
	GPIOR1 |= (1 << 0);
	if (GPIOR1 == 0xFF)GPIOR1 &= ~(1 << 0);
}
void SHIFT() {
	SHIFT1();
	SHIFT0();
	PINB |= (1 << 2);
	PINB |= (1 << 2);
	//read switches
	SW_read();
}
void SW_read() {
	//reads switches
	byte changed;
	byte read;
	byte sw;

	for (byte i = 0; i < 5; i++) {
		if (bitRead(GPIOR1, i) == false) {
			read = PINC;
			changed = read ^ SW_last[i];
			if (changed > 0) {
				SW_last[i] = read;
				/*
				Serial.println("-------");
				Serial.print("ch: ");
				Serial.println(changed);

				Serial.print("last: ");
				Serial.println(SW_last[i]);

				Serial.print("read: ");
				Serial.println(read);
*/
				for (byte b = 0; b < 8; b++) {
					if (bitRead(changed, b) == true & bitRead(read, b) == false) SW_exe((i * 4) + (b));
				}
			}
		}
	}
}
void LED_exe(byte mode) {
	switch (mode) {
	case 0:
		PORTB &= ~(1 << 4);
		PORTB |= (1 << 5);
		break;
	case 1:
		PORTB &= ~(1 << 5);
		PORTB |= (1 << 4);

		break;
	case 2:
		PORTB |= (1 << 4);
		PORTB |= (1 << 5);
		break;
	}
}
void LED_timer() {
	//timer for slow effects, starts in program modes 1 and 2 
	if (millis() - LED_time > 20) { //calls every 20ms
		LED_time = millis();
		LED_blink();
	}
}

void LED_blink() {
	//creates all kind of flashing leds effects
	
	switch (LED_mode) {
	case 10: //confirm switch mode dual, green led flash 4x
		LED_count[0]++;
		if (LED_count[0] == 3) {
			PORTB |= (1 << 5);
		}

		if (LED_count[0] == 6) {
			PORTB &= ~(1 << 5);
			LED_count[1]++;
			LED_count[0] = 0;
			if (LED_count[1] > 8) {
				LED_count[1] = 0;
				LED_mode = 0;
			}
				
		}
		break;
	case 11: //confirm switch mode single, green led flash once
		LED_count[0]++;
		if (LED_count[0] == 5) PORTB |= (1 << 5);
		if (LED_count[0] == 50) {
			PORTB &= ~(1 << 5);
			LED_count[0] = 0;
			LED_mode = 0;
		}	
	
		break;
	default:
		//do nothing
		break;
	}

}
void SW_exe(byte sw) {
	if (sw == 16) {
		COM_mode++;
		COM_reg |= (1 << 1); //enable ledtimer
		if (COM_mode > 2) {
			COM_mode = 0;
			COM_reg &= ~(1 << 1); //disable Led timer
			MEM_change(); //store made changes
		}
		LED_exe(COM_mode);
	}
	else {
		switch (COM_mode) {
		case 0:
			SW_servo(sw);
			break;
		case 1:
			SW_mode1(sw);
			break;
		case 2:
			SW_mode2(sw);
			break;
		}
	}
	Serial.print("switch: ");
	Serial.println(sw);
	//Serial.println("");
}
void SW_mode1(byte sw) {
	byte bit = sw;
	if (bit > 3)bit = bit - 4;
	//handles switche in program mode 1
	//servo is last by push buttons controlled servo
	switch (sw) {
	case 0: //start servo
		SER_reg[SER_last] ^= (1 << 2); //toggle direction
		SER_reg[SER_last] |= (1 << 1); //set request for start
		break;
	case 10: //switch mode
		SER_swm ^= (1 << SER_last); //toggle swich mode
		LED_mode = 10;
		if (bitRead(SER_swm, SER_last) == true)LED_mode = 11;
		break;
	}

}
void SW_mode2(byte sw) {
	//handles switches in program mode 2

}
void SW_servo(byte sw) {
	//handles switches in program mode 0
	boolean dir;
	if (sw < 4) {
		SER_last = sw;
		dir = false;
	}
	else if (sw < 8) {
		SER_last = sw - 4;
		dir = true;
	}
	else if (sw < 12) {
		SER_last = sw - 4;
		dir = false;
	}
	else {
		SER_last = sw - 8;
		dir = true;
	}
	SER_start(SER_last, dir);
}
void SHIFT0() {
	for (byte i = 7; i < 8;) {
		PORTB &= ~(1 << 0);
		if (bitRead(GPIOR0, i) == true) PINB |= (1 << 0);
		PINB |= (1 << 1);
		PINB |= (1 << 1);
		i--;
	}
}
void SHIFT1() {
	for (byte i = 7; i < 8;) {
		PORTB &= ~(1 << 0);
		if (bitRead(GPIOR1, i) == true) PINB |= (1 << 0);
		PINB |= (1 << 1);
		PINB |= (1 << 1);
		i--;
	}
}

void CLK_exe() {//called from loop

	//Serial.println(SH_byte[0] + SH_byte[1]);	

	/*


	//servotesten met servo 0
	//SER_reg[0] |= (1 << 0);
	SER_reg[0] ^= (1 << 7); //toggle richting

	if (bitRead(SER_reg[0], 7) == true) {
		SER_goal[0] = 1500;
	}
	else {
		SER_goal[0] = 1501;
	}
*/


/*

//looplicht
	if (bitRead(SH_byte[0], 7) == true) {
		SH_byte[0] = 0;
		SH_byte[1] = 1;
	}
	else {
		if (SH_byte[0] == 0) {
			SH_byte[1] = SH_byte[1] << 1;
		}
		else {
			SH_byte[0] = SH_byte[0] << 1;
		}
	}
	if (SH_byte[0] + SH_byte[1] == 0) 	SH_byte[0] = 1;

*/

/*
//timer 2 testen
SER_reg[0] ^=(1 << 6);
if (bitRead(SER_reg[0], 6) == true) {
	TCNT2 = 0;
	TCCR2B = 7;
	PORTB |= (1 << 4);
	//TIMSK2 &= ~(1 << 1); //disable interupts
}
else {
	TCCR2B = 0;
}
*/

}
void loop() {
	DEK_DCCh();
	if (bitRead(COM_reg, 1) == true)LED_timer(); //enabled in program modes 1 and 2, GPIOR2 ???
	//SHIFT();

	/*

	//slowtimer
	if (millis() - CLK_time > 500) {
		//CLK_exe();
		CLK_time = millis();
	}
	*/
}
