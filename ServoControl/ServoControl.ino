/*
 Name:		SerVoControl V3.01
 Created:	20nov2011
 Author:	Rob Antonisse

 Versions:
 V1.01 domo development
 V1.02 16-6-2019
	bugs fixed:
	PortC added pull ups to all ports, solves random movements of switch status
	PortB 0 and PortB 1 switched, wrong in eagle design, fixed.
	led blink counters no reset after factory reset

	V2.0 16aug2021
Als knop is ingedrukt tijdens powerup, wordt het een chaos veroorzaakt door ongewenste schakelaar acties.
In Shift() counter toegevoegd die disabled switchs acties in powerup

V3.01  20november2021
DCC decoder gewisseld voor de NMRA decoder heeft 1622 meer bytes nodig, en 1 byte minder geheugen

*/

#include <NmraDcc.h>
#include <FastLED.h>
#include <EEPROM.h>

NmraDcc  Dcc;

//macro
#define fastled GPIOR2 |=(1<<2);

# define Ldef 18000 //left postion default value
# define Rdef 30000 //right position default value
# define Sdef 500 //speed servo default value
# define Pdef 24000 //Centre position default value
# define LMdef 21000 //position left centre
# define RMdef 27000 //position right centre
//colors 


#define red 0x0A0000*MEM_bright
#define green 0x000A00*MEM_bright
#define blue 0x00000A*MEM_bright
#define yellow 0x070400 *MEM_bright
#define grey 0x020202*MEM_bright/2
#define pink 0x0A000A*MEM_bright
#define lightgreen 0x010705*MEM_bright
#define oranje 0x090402*MEM_bright

//declarations common
byte COM_mode;
byte MEM_reg; //EEPROM #105 register with to be restored settings
byte MEM_count;
unsigned int MEM_offset;
byte MEM_bright = 1;

byte COM_dcc; //basic adres DCC receive
byte SH_bitcount;
byte SH_bytecount;
byte SH_fase;
//declarations servo
byte servo; //adresses current handled servo in SER_exe
unsigned int SER_l[8]; //EEPROM 10~25 puls width left
unsigned int SER_r[8]; //EEPROM 30~45 puls width right 32000=maximum? 24000 = centre?
unsigned int SER_lm[8]; //EEPROM 70-86
unsigned int SER_rm[8]; //EEPROM 110-126
unsigned int SER_position[8]; //EEPROM 130~146 stored last position
unsigned int SER_target[8]; //de te bereiken positie
byte SER_dir[8]; //EEPROM 0~7 startup direction 0=l 1=lm 2=rm 3=r 0xff=midden
byte SER_count[8];
unsigned int SER_speed[8];//EEPROM 50-66 motion speed of servo
byte SER_reg[8]; //register byte
byte SER_last; //holds last controlled servo (not DCC controlled)
byte SER_swm; // EEPROM 100; switch mode for the servo mono (default) true, dual false

byte SW_last[8];
byte COM_maxservo = 4; //not used??
volatile byte sc;

CRGB pix[9];
byte flc; //fastled count

byte LED_mode;
unsigned long LED_time;
int LED_count[3]; //two counters for led effects and booleans in blink
byte  SW_count = 0;

void setup() {
	Serial.begin(9600);
	//constructer NMRA decoder
	Dcc.pin(0, 2, 1); //interrupt number 0; pin 2; pullup to pin2
	Dcc.init(MAN_ID_DIY, 10, 0b10000000, 0); 
	//bit7 true maakt accessoire decoder, bit6 false geeft decoder adres of dccadres

	DDRB |= (1 << 3); //pin 11 OE (output enabled) van de shifts
	PORTB |= (1 << 3);

	DDRB |= (1 << 0); //pin 8 serial data out, set as output
	DDRB |= (1 << 1); //pin 9 Rclock, latch set as output
	DDRB |= (1 << 2); //pin 10 shift clock set as output	
	DDRC = 0x0;
	PORTC = 0xFF; //pull-up resistors to port C

	//DeKoder part, interrupt on PIN2
	//DEK_Tperiode = micros();
	EICRA |= (1 << 0);//EICRA – External Interrupt Control Register A bit0 > 1 en bit1 > 0 (any change)
	EICRA &= ~(1 << 1);	//bitClear(EICRA, 1);
	EIMSK |= (1 << INT0);//enable interrupt
	TCCR1A = 0;
	TIMSK1 |= (1 << 1); //enable interupt
	MEM_init();
	LED_mode = 0;
	COM_mode = 0;

	//fastled part must be set after reading EEPROM
	FastLED.addLeds<WS2812, 3, RGB>(pix, 9);
	if (bitRead(MEM_reg, 3) == true) {
		FastLED.addLeds<WS2812, 4, GRB>(pix, 8);
	}
	else {
		FastLED.addLeds<WS2812, 4, RGB>(pix, 8);
	}
	SHIFT();
	PORTB &= ~(1 << 3);
}
void MEM_init() {
	//runs once in startup and part of factory settings reload
	unsigned int ea; //ea=eeprom adres
	//reads and sets initial value from eeprom

	SER_swm = EEPROM.read(100); //switch modes for servo's
	COM_dcc = EEPROM.read(101); //dcc decoder adres (default 255)
	MEM_bright = EEPROM.read(102); //helderheid fastled
	if (MEM_bright > 20)MEM_bright = 4;
	MEM_reg = EEPROM.read(105);
	//Left position value array
	ea = 10; //start adress left
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_l[i]);
		if (SER_l[i] == 0xFFFF) {
			SER_l[i] = Ldef;
			EEPROM.put(ea, SER_l[i]); //put uses update only changed bits will be over written
		}
		ea = ea + 2;
	}
	//right position value array 
	ea = 30; //start adres right
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_r[i]);
		if (SER_r[i] == 0xFFFF) {
			SER_r[i] = Rdef;
			EEPROM.put(ea, SER_r[i]); //put uses update only changed bits will be over written
		}
		ea = ea + 2;
	}
	//speed value array 
	ea = 50;
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_speed[i]);
		if (SER_speed[i] == 0xFFFF) {
			SER_speed[i] = Sdef;
			EEPROM.put(ea, SER_speed[i]); //put uses update only changed bits will be over written
		}
		ea = ea + 2;
	}
	//left centre position value array 
	ea = 70; //start adres
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_lm[i]);
		if (SER_lm[i] == 0xFFFF) {
			SER_lm[i] = LMdef;
			EEPROM.put(ea, SER_lm[i]); //put uses update only changed bits will be over written
		}
		ea = ea + 2;
	}
	//right centre position value array 
	ea = 110; //start adres
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_rm[i]);
		if (SER_rm[i] == 0xFFFF) {
			SER_rm[i] = RMdef;
			EEPROM.put(ea, SER_rm[i]); //put uses update only changed bits will be over written
		}
		ea = ea + 2;
	}
	/*
	To ensure that servo's will not start with uncontrolled fast movements, it is needed to store the current position of the servo's at all times.
	Routine here divides the 30bytes needed for this over 300 EEPROM bytes improving endurance of the EEPROM memorie to 1 million write cycles.
	Further the update of the EEPROM will take place when all servo's are stopped to reduce the writing cycles further more.
	If Arduino is turned on average 3 times a day, EEPROM should last for more the 100 years.
	*/

	MEM_count = EEPROM.read(106);
	if (MEM_count == 0xFF)MEM_count = 0; //initial value after eeprom clear
	MEM_offset = 30 * MEM_count;

	//directions, positions
	ea = 200 + MEM_offset;
	for (byte i = 0; i < 8; i++) {
		SER_dir[i] = EEPROM.read(ea);
		if (SER_dir[i] == 0xFF)SER_dir[i] = 4;
		//set pixels colors
		servo = i;
		PIX_set(servo);
		ea++;
	}
	//(stored) last position restore
	ea = 209 + MEM_offset;
	//Serial.println("INIT");
	for (byte i = 0; i < 8; i++) {
		EEPROM.get(ea, SER_position[i]);
		if (SER_position[i] == 0xFFFF) {
			SER_position[i] = Pdef;
		}
		ea = ea + 2;
	}

	MEM_count++;
	if (MEM_count > 10)MEM_count = 0; //0~9
	MEM_offset = 30 * MEM_count; //new offset value
	EEPROM.update(106, MEM_count); //Store new mem_count
	MEM_position(); //store direction and positions in new EEPROM locations

	pix[8] = grey;
	fastled;
}
void MEM_position() {
	unsigned int ea;
	ea = 200 + MEM_offset;
	for (byte i = 0; i < 8; i++) {
		EEPROM.update(ea, SER_dir[i]);
		ea++;
	}
	//store positions
	ea = 209 + MEM_offset;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_position[i]);
		ea = ea + 2;
	}
}
void MEM_factory() {
	//resets all EEPPROM to 0xFF
	int maxmem;
	maxmem = EEPROM.length();
	for (int i = 0; i < maxmem; i++) {
		EEPROM.update(i, 0xFF);
	}
	//reload pre defined values
	MEM_init();
}
void MEM_change() {
	int ea;
	//checks for changes in Memorie
	EEPROM.update(100, SER_swm); //switch modes
	//update left position
	EEPROM.update(102, MEM_bright);
	EEPROM.update(105, MEM_reg);

	ea = 10;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_l[i]);
		ea = ea + 2;
	}
	//update right position
	ea = 30;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_r[i]);
		ea = ea + 2;
	}
	//update speed
	ea = 50;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_speed[i]);
		ea = ea + 2;
	}
	//update left centre position
	ea = 70;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_lm[i]);
		ea = ea + 2;
	}
	//update right centre position
	ea = 110;
	for (byte i = 0; i < 8; i++) {
		EEPROM.put(ea, SER_rm[i]);
		ea = ea + 2;
	}
}
ISR(TIMER1_COMPA_vect) {
	SER_stop();
}
void notifyDccAccTurnoutBoard(uint16_t BoardAddr, uint8_t OutputPair, uint8_t Direction, uint8_t OutputPower) {
	//Call Back van NmraDcc library
	//called als CV29 bit6 = false decoderadres,channel,poort,onoff (zie setup 'init')
	//interupt mask register cleared in startmotor() en set in stop() om DCC te disabelen als motor draait. ((nog)niet in homen)
	APP_DCC(BoardAddr, OutputPair + 1, Direction, OutputPower);
}
void APP_DCC(int decoder, int channel, boolean port, boolean onoff) {
	int adres;
	adres = ((decoder - 1) * 4) + channel;
	//Applications 
	//APP_Monitor(adres, decoder, channel, port, onoff);
	APP_function(adres, decoder, channel, port, onoff);
}
void APP_Monitor(int adres, int decoder, int channel, boolean port, boolean onoff) {

	Serial.print("Adres: ");
	Serial.print(adres);
	Serial.print("(");
	Serial.print(decoder);
	Serial.print("-");
	Serial.print(channel);
	Serial.print("), ");
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
	Serial.println("");
}
void APP_function(int adres, int decoder, int channel, boolean port, boolean onoff) {
	unsigned int dcc; //adres to be calculated?

	if (bitRead(GPIOR2, 7) == true) {//instellen adres
		if (decoder < 255) { //possible values 0-254
			COM_dcc = decoder;
			LED_mode = 4;
			COM_mode = 0;		}
	}
	else {
		dcc = (adres - (COM_dcc - 1) * 4) - 1;
		
		if (dcc < 16) {
			//if (type == false) { //switch command
				if (dcc < 8) {
					if (port == true) {
						SER_start(dcc, 1);
					}
					else {
						SER_start(dcc, 2);
					}
				}
				else if (dcc < 16 & bitRead(MEM_reg, 2) == false) {
					if (port == true) {
						SER_start(dcc - 8, 4);
					}
					else {
						SER_start(dcc - 8, 5);
					}
				}
		//	}
		//	else { //CV command
		//	}
		}
	}

}
void SER_reset() {
	//resets last controlled servo
	SER_l[SER_last] = Ldef;
	SER_r[SER_last] = Rdef;
	SER_lm[SER_last] = LMdef;
	SER_rm[SER_last] = RMdef;
	SER_speed[SER_last] = Sdef;
	SER_swm |= (1 << SER_last);
	SER_count[0] = 0;
	pix[7] = red;
}
void SER_start(byte sv, byte target) {
	if (bitRead(SER_reg[sv], 0) == false) SER_reg[sv] |= (1 << 1); //request for run
	switch (target) {
	case 0: //toggle direction l<>r
		SER_reg[sv] ^= (1 << 2); //direction l<>r
		if (bitRead(SER_reg[sv], 2) == false) {
			SER_target[sv] = SER_r[sv];
			SER_dir[sv] = 3;
		}
		else {
			SER_target[sv] = SER_l[sv];
			SER_dir[sv] = 0;
		}
		break;
	case 1: //set direction l
		SER_reg[sv] |= (1 << 2);
		SER_target[sv] = SER_l[sv];
		SER_dir[sv] = 0;
		break;
	case 2: //set direction r
		SER_reg[sv] &= ~(1 << 2);
		SER_target[sv] = SER_r[sv];
		SER_dir[sv] = 3;
		break;
	case 3: //toggle direction lm<>rm
		SER_reg[sv] ^= (1 << 3); //direction lm<<>>rm
		if (bitRead(SER_reg[sv], 3) == false) {
			SER_target[sv] = SER_lm[sv];
			SER_dir[sv] = 1;
		}
		else {
			SER_target[sv] = SER_rm[sv];
			SER_dir[sv] = 2;
		}
		break;
	case 4: //direction lm
		SER_reg[sv] |= (1 << 3);
		SER_target[sv] = SER_lm[sv];
		SER_dir[sv] = 1;
		break;
	case 5: //direction rm
		SER_reg[sv] &= ~(1 << 3);
		SER_target[sv] = SER_rm[sv];
		SER_dir[sv] = 2;
		break;
	}
	switch (COM_mode) {
	case 0:
		pix[sv] = grey;
		break;
	case 1:
		pix[0] = grey;
		break;
	case 2:
		break;
	}
	fastled;
}
void SER_stop() { //called from ISR//
	GPIOR0 &= ~(1 << servo); //reset puls servo
	TCCR1B = 0; //stop timer 2	
	SHIFT();
	SER_run();
}
void SER_run() {
	byte count;
	byte temp;
	GPIOR2 ^= (1 << 0);
	TCNT1 = 0;
	if (bitRead(GPIOR2, 0) == true) { //start servo	
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
void PIX_set(byte srv) {
	byte px = 0;
	if (COM_mode == 0)px = srv;
	switch (SER_dir[srv]) {
	case 0:
		pix[px] = green;
		break;
	case 1:
		pix[px] = yellow;
		break;
	case 2:
		pix[px] = blue;
		break;
	case 3:
		pix[px] = red;
		break;
	case 4:
		pix[px] = grey;
		break;
	}
}
void SER_set() { //called from SER_run 	
	sc = 0;
	if (bitRead(SER_reg[servo], 0) == true) { //servo runs
		if (SER_position[servo] == SER_target[servo]) {
			SER_count[servo]++;
			if (SER_count[servo] > 4) {
				SER_reg[servo] &= ~(1 << 0); //stop servo
				SER_count[servo] = 0;
				//set pixel			
				if (COM_mode != 2) {
					PIX_set(servo);
					fastled;
				}
				//check if any servo is running
				for (byte i = 0; i < 8; i++) {
					if (bitRead(SER_reg[i], 0) == true) {
						sc++;
						i = 10;
					}
				}
				if (sc == 0) MEM_position(); //store position of the servo's
			}
		}
		else if (SER_position[servo] > SER_target[servo]) {

			if (SER_position[servo] - SER_target[servo] > SER_speed[servo]) {
				SER_position[servo] = SER_position[servo] - SER_speed[servo];
			}
			else {
				SER_position[servo] = SER_target[servo];
			}
		}
		else if (SER_position[servo] < SER_target[servo]) {
			if (SER_target[servo] - SER_position[servo] > SER_speed[servo]) {
				SER_position[servo] = SER_position[servo] + SER_speed[servo];
			}
			else {
				SER_position[servo] = SER_target[servo];
			}
		}
		/*

		switch (bitRead(SER_reg[servo], 2)) { //direction
		case false: //left
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

		case true: //right
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



	*/
	}
	else { //servo not active
		//FastLED.show();
		if (bitRead(SER_reg[servo], 1) == true) {

			for (byte i = 0; i < 8; i++) {
				if (bitRead(SER_reg[i], 0) == true)sc++;
			}
			//limit active servo's standard 4
			if (sc < COM_maxservo) {
				SER_reg[servo] |= (1 << 0);
				SER_reg[servo] &= ~(1 << 1);
			}
		}
	}

	//set registers for switch scanning
	GPIOR1 = (GPIOR1 << 1);
	GPIOR1 |= (1 << 0);
	if (GPIOR1 == 0xFF)GPIOR1 &= ~(1 << 0);
}
void SHIFT() {
	SHIFT1();
	SHIFT0();
	PINB |= (1 << 1); //1 en 2 verwisseld
	PINB |= (1 << 1);


	//tijdens powerup geen switches lezen (V2.0)
	if (SW_count > 100) {
		SW_read();//read switches
	}
	else {
		SW_count++;
	}

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
void LED_timer() {
	//timer for slow effects, starts in program modes 1 and 2 
	if (millis() - LED_time > 20) { //calls every 20ms
		LED_time = millis();
		LED_blink();
	}
}
void LED_pix(byte mode) {
	//sets initial colors to pixels in programmodes
	switch (mode) {
	case 0:
		for (byte i = 0; i < 8; i++) {
			PIX_set(i);
		}
		pix[8] = grey;
		break;
	case 1: //program 1
		PIX_set(SER_last);
		pix[1] = lightgreen;
		pix[2] = lightgreen;
		pix[3] = pink;
		pix[4] = pink;
		if (bitRead(SER_swm, SER_last) == true) {
			pix[5] = yellow;
		}
		else {
			pix[5] = blue;
		}
		pix[6] = oranje;
		pix[7] = red;
		if (MEM_bright == 0) {
			pix[8] = 0x0000A0;
		}
		else {
			pix[8] = blue;
		}
		break;
	case 2: //program 2
		//pix0 2 or 4 positions
		if (bitRead(MEM_reg, 1) == true) {
			pix[0] = lightgreen;
		}
		else {
			pix[0] = oranje;
		}

		//pix1 standen knoppen
		byte sr;
		for (byte i = 0; i < 8; i++) {
			if (SER_swm == 0xFF) {
				pix[1] = lightgreen;
			}
			else if (SER_swm == 0x0) {
				pix[1] = oranje;
			}
			else {
				pix[1] = pink;
			}
		}

		pix[2] = 0x0;

		if (bitRead(MEM_reg, 3) == true) {
			pix[3] = green;
		}
		else {
			pix[3] = red;
		}

		pix[4] = oranje;

		if (bitRead(MEM_reg, 2) == true) {
			pix[5] = lightgreen;
		}
		else {
			pix[5] = oranje;
		}

		pix[6] = blue;
		pix[7] = red;
		if (MEM_bright == 0) {
			pix[8] = 0xA00000;
		}
		else {
			pix[8] = red;
		}
		break;
	}
	fastled;
}
void LED_blink() {
	//creates all kind of flashing leds effects
	switch (LED_mode) {
	case 0:
		//do nothing
		break;
	case 1: //factory reset wait for confirmation
		LED_count[0]++;
		if (LED_count[0] == 4) {
			pix[7] = red;
			fastled;
		}
		if (LED_count[0] == 8) {
			LED_count[0] = 0;
			pix[7] = blue;
			fastled;
		}

		break;
	case 2: //servo reset wait for confirmation
		LED_count[0]++;
		if (LED_count[0] == 3) {
			pix[7] = red;
			fastled;
		}

		if (LED_count[0] == 4) {
			LED_count[0] = 0;
			pix[7] = 0x0;
			fastled;
		}
		break;
	case 3://DCC decoder adress set, wait for command
		LED_count[0]++;
		if (LED_count[0] == 10) {
			pix[6] = yellow;
			fastled;
		}
		if (LED_count[0] == 20) {
			pix[6] = pink;
			LED_count[0] = 0;
			fastled;
		}
		break;
	case 4: //confirm DCC adres received
		LED_count[0]++;
		if (LED_count[0] == 10) {
			pix[6] = grey;
			fastled;
		}
		if (LED_count[0] == 15) {
			pix[6] = green;
			fastled;
		}
		if (LED_count[0] == 40) {
			pix[6] = 0x0;
			fastled;
		}
		if (LED_count[0] == 50) {
			LED_count[0] = 0;
			LED_count[1] = 0;
			LED_mode = 0;
			GPIOR2 &= ~(1 << 7);//reset flag
			COM_mode = 0;
			LED_pix(0);
			EEPROM.update(101, COM_dcc);  //dcc decoder adress

		}
		break;
	case 10://mirror positions, flash red pix 6
		LED_count[0]++;
		if (LED_count[0] == 1)pix[6] = 0x0;
		if (LED_count[0] == 4) {
			LED_count[0] = 0;
			pix[6] = blue;
			LED_count[1]++;
			if (LED_count[1] > 4) {
				LED_count[1] = 0;
				LED_mode = 0;
				pix[6] = oranje;
			}
		}
		fastled;
		break;
	case 11:
		LED_count[0]++;
		if (LED_count[0] == 2) {
			pix[LED_count[2]] = 0x0;
			fastled;
		}
		if (LED_count[0] == 4) {
			if (LED_count[2] < 3) {
				pix[LED_count[2]] = lightgreen;
			}
			else {
				pix[LED_count[2]] = pink;
			}
			LED_count[0] = 0;
			LED_count[1]++;
			if (LED_count[1] > 2) {
				clearcounts();
				//LED_count[1] = 0;
				//LED_count[2] = 0;
				//LED_mode = 0;
			}
			fastled;
		}
		break;
	default:
		//do nothing
		break;
	}
}
void SW_exe(byte sw) {
	if (sw == 16) { //program button
		clearcounts();
		COM_mode++;
		if (COM_mode > 2)COM_mode = 0;

		switch (COM_mode) {
		case 0:
			GPIOR2 &= ~B11100010;
			LED_mode = 0;
			MEM_change(); //store made changes
			LED_pix(0);
			break;
		case 1:
			LED_pix(1);
			GPIOR2 |= (1 << 1); //enable ledtimer
			break;
		case 2:
			LED_pix(2);
			break;
		default:
			break;
		}
	}
	else { //0~15 schakelaars ingedrukt.

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
	fastled;
}
void SW_mode1(byte sw) {//handles switche in program mode 1, servo is last by push buttons controlled servo
	byte td = SER_reg[SER_last]; //store register of active servo
	byte bit = sw;
	int temp;
	if (bit > 3)bit = bit - 4;
	switch (sw) {
	case 0: //start servo
		if (bitRead(MEM_reg, 1) == true) { //twee standen servo
			if (SER_dir[SER_last] != 0) {
				temp = 1;
			}
			else {
				temp = 2;
			}
		}
		else { //4 standen servo
			SER_dir[SER_last]++;
			if (SER_dir[SER_last] > 3)SER_dir[SER_last] = 0;
			switch (SER_dir[SER_last]) {
			case 0:
				temp = 1;
				break;
			case 1:
				temp = 4;
				break;
			case 2:
				temp = 5;
				break;
			case 3:
				temp = 2;
				break;
			}
		}
		SER_start(SER_last, temp);
		break;
	case 1: //move counter clockwise
		SER_pchng(200);
		SER_reg[SER_last] |= (1 << 1); //request servo start
		if (LED_mode == 0) {
			LED_mode = 11;
			LED_count[2] = 1;
		}

		break;
	case 2: //move clockwise
		SER_pchng(-200);
		SER_reg[SER_last] |= (1 << 1); //request servo start
		if (LED_mode == 0) {
			LED_mode = 11;
			LED_count[2] = 2;
		}

		break;

	case 3: //decrease speed
		temp = SER_speed[SER_last] / 10;
		if (SER_speed[SER_last] > temp) SER_speed[SER_last] = SER_speed[SER_last] - temp;
		if (LED_mode == 0) {
			LED_mode = 11;
			LED_count[2] = 3;
		}

		break;
	case 8: //increase speed
		temp = SER_speed[SER_last] / 10;
		if (SER_speed[SER_last] + temp < 0xFFFF)SER_speed[SER_last] = SER_speed[SER_last] + temp;
		if (LED_mode == 0) {
			LED_mode = 11;
			LED_count[2] = 4;
		}

		break;
	case 9: //switch mode
		SER_swm ^= (1 << SER_last); //toggle swich mode
		//LED_mode = 10;
		if (bitRead(SER_swm, SER_last) == true) {
			pix[5] = yellow;
		}
		else {
			pix[5] = blue;
		}
		break;
	case 10: //mirror position l<>r lm<>rm
		LED_mode = 10;
		temp = SER_l[SER_last];
		SER_l[SER_last] = SER_r[SER_last];
		SER_r[SER_last] = temp;
		temp = SER_lm[SER_last];
		SER_lm[SER_last] = SER_rm[SER_last];
		SER_rm[SER_last] = temp;

		break;
	case 11: //reset this servo to default values
		if (bitRead(GPIOR2, 6) == false) {
			GPIOR2 |= (1 << 6);
			LED_mode = 2; //flashing only green
		}
		else {
			GPIOR2 &= ~(1 << 6);
			LED_mode = 0;
			SER_reset();
		}
		break;
	}
	fastled;
}
void SW_mode2(byte sw) {
	//handles switches in program mode 2
	switch (sw) {
	case 0: //Keuze 2 standen of 4 standen afstelling
		MEM_reg ^= (1 << 1); //
		if (bitRead(MEM_reg, 1) == true) {
			pix[0] = lightgreen;
		}
		else {
			pix[0] = oranje;
		}
		fastled;
		break;

	case 1: //toggle switch mode all servo's		
		MEM_reg ^= (1 << 0); //toggle switch mode
		if (bitRead(MEM_reg, 0) == true) {
			SER_swm = 0xFF;
			pix[1] = lightgreen; //mono mode button toggles direction
		}
		else {
			SER_swm = 0x00; //duo mode button set one direction (2 buttons needed)
			pix[1] = oranje;
		}
		fastled;
		break;
	case 3://set color scene for external neopixels
		MEM_reg ^= (1 << 3);
		if (bitRead(MEM_reg, 3) == true) {
			pix[3] = green;
		}
		else {
			pix[3] = red;
		}
		break;
	case 8: //brightness of fastled pixels
		if (MEM_bright > 15) {
			MEM_bright = 0;
		}
		else {
			MEM_bright++;
		}
		LED_pix(2);
		break;
	case 9: //set two decoder adresses or 4 decoder adresses (4 positions by DCC commmands
		MEM_reg ^= (1 << 2);
		if (bitRead(MEM_reg, 2) == true) {
			pix[5] = lightgreen;
		}
		else {
			pix[5] = oranje;
		}
		break;

	case 10: //setting DCC decoder adres
		GPIOR2 |= (1 << 7);
		LED_mode = 3;
		break;
	case 11: //factory reset, needs 2x press
		if (bitRead(GPIOR2, 6) == false) {
			GPIOR2 |= (1 << 6);
			LED_mode = 1;
		}
		else {
			GPIOR2 &= ~(1 << 6);
			MEM_factory();
			LED_mode = 0;
			COM_mode = 0;
			LED_pix(0);
			clearcounts();
		}
		break;
	}
}
void clearcounts() {
	LED_count[0] = 0;
	LED_count[1] = 0;
	LED_count[2] = 0;
	LED_mode = 0;
	GPIOR2 &= ~(1 << 6);
}
void SER_pchng(int mut) {

	switch (SER_dir[SER_last]) {
	case 0:
		SER_l[SER_last] = SER_l[SER_last] + mut;
		SER_target[SER_last] = SER_l[SER_last];
		break;
	case 1:
		SER_lm[SER_last] = SER_lm[SER_last] + mut;
		SER_target[SER_last] = SER_lm[SER_last];
		break;
	case 2:
		SER_rm[SER_last] = SER_rm[SER_last] + mut;
		SER_target[SER_last] = SER_rm[SER_last];
		break;
	case 3:
		SER_r[SER_last] = SER_r[SER_last] + mut;
		SER_target[SER_last] = SER_r[SER_last];
		break;
	}
}
void SW_servo(byte sw) {
	//handles switches in program mode 0
	byte target;
	if (sw < 4) {
		SER_last = sw;
		if (bitRead(SER_swm, SER_last) == true) {
			target = 0; //toggle direction
		}
		else {
			target = 2;
		}
	}
	else if (sw < 8) {
		SER_last = sw - 4;
		if (bitRead(SER_swm, SER_last) == true) {
			target = 3;
		}
		else {
			target = 1;
		}
	}
	else if (sw < 12) {
		SER_last = sw - 4;
		if (bitRead(SER_swm, SER_last) == true) {
			target = 0; //toggle direction
		}
		else {
			target = 2;
		}
	}
	else {
		SER_last = sw - 8;
		if (bitRead(SER_swm, SER_last) == true) {
			target = 3;
		}
		else {
			target = 1;
		}
	}
	SER_start(SER_last, target);
}
void SHIFT0() {
	for (byte i = 7; i < 8;) {
		PORTB &= ~(1 << 0);
		if (bitRead(GPIOR0, i) == true) PINB |= (1 << 0);
		PINB |= (1 << 2); //was 1
		PINB |= (1 << 2);
		i--;
	}
}
void SHIFT1() {
	for (byte i = 7; i < 8;) {
		PORTB &= ~(1 << 0);
		if (bitRead(GPIOR1, i) == true) PINB |= (1 << 0);
		PINB |= (1 << 2);
		PINB |= (1 << 2);
		i--;
	}
}
void loop() {
	flc++;
	Dcc.process();
	//DEK_DCCh();

	if (bitRead(GPIOR2, 1) == true)LED_timer(); //enabled in program modes 1 and 2, GPIOR2 ???
	//SHIFT();
	if (bitRead(GPIOR2, 2) == true & flc == 0) {
		FastLED.show();
		GPIOR2 &= ~(1 << 2);
	}
}
