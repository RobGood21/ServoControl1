/*
 Name:		ServoControl.ino
 Created:	4/30/2019 3:59:50 PM
 Author:	gebruiker
*/

//declarations
byte SH_byte[2]; //2 bytes to be shifted out
byte SH_bitcount;
byte SH_bytecount;
byte SH_fase;

unsigned long CLK_time;


void setup() {
	DDRD |= (1 << 7); //pin7 OE (output enabled) van de shifts
	Serial.begin(9600);
	//shiftregisters init
	DDRD |= (1 << 4); //pin4 serial data out, set as output
	DDRD |= (1 << 5); //pin5 Rclock, latch set as output
	DDRD |= (1 << 6); //pin6 shift clock set as output

	//PORTD |= (1 << 7); //set disable output	
}

void SHIFT() {
	//Serial.println(SH_fase);
	switch (SH_fase) {
	case 0: //start shift process
		PORTD &= ~(1 << 5); //store 
		PORTD &= ~(1 << 6); //shift
		SH_fase = 1;
		break;
	case 1:
		//load bit
		if (bitRead(SH_byte[SH_bytecount], SH_bitcount) == true) {
			PORTD |= (1 << 4);
		}
		else {
			PORTD &= ~(1 << 4);  //ser data	
		}
		SH_fase = 2;
		break;
	case 2:
		//shift high
		PORTD |= (1 << 6);
		SH_fase = 3;
		break;
	case 3:
		//shift low, load next bit
		SH_fase = 1;
		PORTD &= ~(1 << 6); //shift low
		SH_bitcount--;
		if (SH_bitcount > 7) {
			SH_bitcount = 7;
			SH_bytecount--;
			if (SH_bytecount > 1) {
				SH_bytecount = 1;  //aantal bytes is hier aan te passen.1=2 2=3 enz
				SH_fase = 4;
			}
		}
		break;
	case 4:
		//lock in shift registers to output
		PORTD |= (1 << 5);
		SH_fase = 0; //restart
		break;
	}
}

void CLK_exe() {//called from loop
	//Serial.println(SH_byte[0] + SH_byte[1]);	


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
}

// the loop function runs over and over again until power down or reset
void loop() {
	SHIFT();

	//slowtimer
	if (millis() - CLK_time > 500) {
		CLK_exe();
		CLK_time = millis();
	}

}
