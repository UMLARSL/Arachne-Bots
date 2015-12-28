#include <EEPROM.h>
#include <cc1101.h>

//motor pins
#define FORWARD_MOTOR 9
#define REVERSE_MOTOR 6
#define LEFT_MOTOR 5
#define RIGHT_MOTOR 3

//radio information
CC1101 cc1101;
byte receiverAddress = 6;
byte syncWord[]= {19, 9};
boolean packet_available = false;


int motor_speed = 35
;

void cc1101signalsInterrupt();

void forward(int motor_speed) ;
void reverse(int motor_speed) ;
void left(int motor_speed) ;
void right(int motor_speed) ;
void stop() ;

void setup()
{
	//serial init
	Serial.begin(38400);
	Serial.println("Start");

	//motor init
	pinMode(FORWARD_MOTOR, OUTPUT);
	pinMode(REVERSE_MOTOR, OUTPUT);
	pinMode(LEFT_MOTOR, OUTPUT);
	pinMode(RIGHT_MOTOR, OUTPUT);
	stop();

	//radio init
	cc1101.init();
	cc1101.setCarrierFreq(CFREQ_915);
	cc1101.setSyncWord(syncWord, false);
	cc1101.setDevAddress(receiverAddress, false);
	cc1101.enableAddressCheck();
	cc1101.setRxState();
	//attachInterrupt(0, cc1101signalsInterrupt, FALLING);

	Serial.println("Initialized");
}

void loop()
{
	/*if(packet_available){
		packet_available = false;
		CCPACKET packet;
		detachInterrupt(0);

		if(cc1101.receiveData(&packet) > 0){
			Serial.print("packet: len");
			Serial.print(packet.length);
			Serial.print(" data1: ");
			Serial.println(packet.data[1]);
			

			//read in as ascii
			switch(packet.data[1]){
			case 115:
				forward(motor_speed);
				break;
			case 119:
				reverse(motor_speed);
				break;
			default:
				stop();
				break;/.;
			}

		}
,nbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb 
		attachInterrupt(0, cc1101signalsInterrupt, FALLING);
	}*/
	forward(motor_speed);
        Serial.println("F");
	delay(500);
	reverse(motor_speed);
        Serial.println("R");
	delay(500);
        forward(0); 
	right(motor_speed);
	delay(500);
        left(motor_speed);
        delay(500);
	stop();
        delay(500);


}
void cc1101signalsInterrupt(){
	// set the flag that a package is available
	packet_available = true;
}

void forward(int motor_speed) {
	analogWrite( FORWARD_MOTOR, motor_speed ) ;
	digitalWrite( REVERSE_MOTOR, LOW ) ;
}

void reverse(int motor_speed) {
	digitalWrite( FORWARD_MOTOR, LOW ) ;
	analogWrite( REVERSE_MOTOR, motor_speed ) ;
}

void left(int motor_speed) {
	analogWrite( LEFT_MOTOR, motor_speed ) ;
	digitalWrite( RIGHT_MOTOR, LOW ) ;
}

void right(int motor_speed) {
	analogWrite(RIGHT_MOTOR, motor_speed);
	digitalWrite( LEFT_MOTOR, LOW ) ;
}

void stop() {
	digitalWrite( FORWARD_MOTOR, LOW ) ;	
	digitalWrite( REVERSE_MOTOR, LOW ) ;
	digitalWrite( LEFT_MOTOR, LOW ) ;
	digitalWrite( RIGHT_MOTOR, LOW ) ;
}
