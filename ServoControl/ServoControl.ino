/*
 Name:		ServoControl.ino
 Created:	4/30/2019 3:59:50 PM
 Author:	gebruiker
*/
//declarations common


//Declaraties deKoder
volatile unsigned long DEK_Tperiode; //laatst gemeten tijd 
volatile unsigned int DEK_duur; //gemeten duur van periode tussen twee interupts
boolean DEK_Monitor = false; //shows DCC commands as bytes
byte DEK_Reg; //register voor de decoder 
byte DEK_Status = 0;
byte DEK_byteRX[6]; //max length commandoos for this decoder 6 bytes (5x data 1x error check)
byte DEK_countPA = 0; //counter for preample
byte DEK_BufReg[12]; //registerbyte for 12 command buffers //bit7= free (false) bit0= CV(true)
byte DEK_Buf0[12];
byte DEK_Buf1[12];
byte DEK_Buf2[12];
byte DEK_Buf3[12];
byte DEK_Buf4[12];
byte DEK_Buf5[12];

//declarations shiftregisters
byte SH_byte[2]; //2 bytes to be shifted out
byte SH_bitcount;
byte SH_bytecount;
byte SH_fase;

//declarations servo
byte servo; //adresses current handled servo in SER_exe
byte SER_l[8]; //puls width left x10 in us
byte SER_r[8]; //puls width right
byte SER_speed[8];//motion speed of servo
byte SER_reg[8]; //register byte
unsigned long SER_time; //timer for pulswidth
unsigned int SER_goal[8]; //goal, position to reach
unsigned long SER_freq; //timer for 50hz servo pulse
unsigned long CLK_time;
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

					  //DeKoder part, interrupt on PIN2
	//DDRD &= ~(1 << 2);//bitClear(DDRD, 2); //pin2 input
	//DDRD |= (1 << 2);

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

	DDRB |= (1 << 4); //pin 4 as output temp led control of interrupt timer 2
	SER_reg[1] |= (1 << 0); //start servo 2
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
	if (DEK_duur > 50) {
		if (DEK_duur < 62) {
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
			if (DEK_duur > 106) {

				if (DEK_duur < 124) { //preferred 118 6us extra space in false bit
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
	//cli();
	//EIMSK &= ~(1 << INT0);
	//SREG &= ~(1 << 7);
	SER_stop();
	//EIMSK |= (1 << INT0);
	SER_run();


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
	while (i < 12) {

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
				if (DEK_byteRX[2] = DEK_byteRX[0] ^ DEK_byteRX[1])DEK_BufCom(false);
				break; //6
			case 6: ///Accessory decoder configuration variable Access Instruction received (CV)
				//in case of CV, handle only write command
				if (bitRead(DEK_byteRX[2], 3) == true && (bitRead(DEK_byteRX[2], 2) == true)) {
					//check errorbyte and make command
					if (DEK_byteRX[5] = DEK_byteRX[0] ^ DEK_byteRX[1] ^ DEK_byteRX[2] ^ DEK_byteRX[3] ^ DEK_byteRX[4])DEK_BufCom(true);
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
	if (n > 12)n = 0;
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
	//APP_Monitor(type, adres, decoder, channel, port, onoff, cv, value);
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
	//executes DCC commands called from COM_exe
	static byte temp;


	if ((temp ^ (adres + port)) > 0) {
		if (adres == 9) {
			if (port == true) {

				//tijdmeting = micros();

				//TCNT2 = 0; //reset counter
				//TIMSK2 |= (1 << 1); //enable interupts
				//PORTB |= (1 << 4);
			}
			else {
				//Serial.println(gemeten);
				//TIMSK2 &= ~(1 << 1); //disable interupts
				//PORTB &= ~(1 << 4);
			}
		}
	}


	if (type == false) { //switch command
		if (adres > 0 & adres < 9) {
			adres--;
			if (port == true) {
				SER_reg[adres] |= (1 << 0);
				//SH_byte[0] |= (1 << adres);
			}
			else {
				SER_reg[adres] &= ~(1 << 0);
				//SH_byte[0] |= (1 << adres);

				//SH_byte[0] &= ~(1 << adres);
			}
			SHIFT();
		}

	}
	else { //CV command

	}
	temp = adres + port;
}

void SER_stop() { //called from ISR//
	SH_byte[0] &= ~(1 << servo); //reset puls servo
	TCCR1B = 0; //stop timer 2
	SHIFT();
	//if (micros() - tijdmeting > 1560) Serial.println(micros() - tijdmeting);
}
void SER_run() {
	GPIOR0 ^= (1 << 0);
	TCNT1 = 0;
	if (bitRead(GPIOR0, 0) == true) { //test start servo
		servo++;
		if (servo > 7) servo = 0;
		if (bitRead(SER_reg[servo], 0) == true) {
			SH_byte[0] |= (1 << servo); //set servo puls	
			SHIFT();
		}
		OCR1A = 24000; // – Output Compare Register A
		TCCR1B = 1; // |= (1 << 0); //set clock no prescaler		
	} 
	else { //pauze interval between servo controlpulses
		OCR1A = 0xFFFA;
		TCCR1B = 2;
	}
	//tijdmeting = micros();	
}
void SHIFT() { //new much faster version

	//EIMSK &= ~(1 << INT0);
	for (byte b = 1; b < 2;) {
		for (byte i = 7; i < 8;) {
			if (bitRead(SH_byte[b], i) == true) {
				PORTB |= (1 << 0); //set pin 8
			}
			else {
				PORTB &= ~(1 << 0);
			}
			PORTB |= (1 << 1);
			PORTB &= ~(1 << 1);
			i--;
		}
		b--;
	}
	PORTB |= (1 << 2);
	PORTB &= ~(1 << 2);
	//EIMSK |= (1 << INT0); //INT0
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

	/*

		if (millis() - SER_freq > 2) { //10*8=80ms = 11,3hz
			SER_freq = millis();
			SER_run();
		}
		else {

		}

	*/
	/*

	//slowtimer
	if (millis() - CLK_time > 500) {
		//CLK_exe();
		CLK_time = millis();
	}
	*/
}
