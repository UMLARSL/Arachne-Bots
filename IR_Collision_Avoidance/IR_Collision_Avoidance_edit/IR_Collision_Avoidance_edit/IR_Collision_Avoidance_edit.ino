//Receives coordinates and waypoints from the IR_Collision_Avoidance panstamp connected to a computer and sends back IR measurements.
//Also implements a movmement algorith to travel to said locations.
//Written by Ben Sullivan

#include "EEPROM.h"
#include "cc1101.h"

//constants
#define LEDOUTPUT        4       // Connect led to digital pin 4
#define RFCHANNEL        0       // Let's use channel 0
#define SYNCWORD1        0xB5    // Synchronization word, high byte
#define SYNCWORD0        0x47    // Synchronization word, low byte
#define SOURCE_ADDR      5       // Sender address
#define DESTINATION_ADDR 4       // Receiver address
int sender_address = 7;
const double pi = 4 * atan( ( double ) 1 );
#define Sigma 1.8 //range of error for theta_rg
#define Mu 15 //range of error for rho
#define SPEED 1
#define TURN_SPEED 127
boolean packet_available = false;

//IR Variable
#define N A0
#define NE A1
#define E A2
#define SE A3
#define S A4
#define SW A5
#define W A6
#define NW A7



//time based
#define txTime 100
unsigned long clock;

//variables
int X_t, Y_t ;
int north, northEast, east, southEast, south, southWest, west, northWest;

int movem;

//radio info
CC1101 cc1101;      // radio object
CCPACKET txPacket;  // packet object


//byte parsing info
unsigned char LSB;
unsigned char MSB;
int byte_parse(unsigned char LSB, unsigned char MSB );//parse bytes used for transmission into data integers
void byte_conversion(int number);//change integers into two bytes to transfer on radio packet

class positionVector
{

public:
	int X;
	int Y;
	double X_i;
	double Y_j;
	int X_d;
	int Y_d;
	double Theta;
	double Magnitude;

        double getMagnitude();
	void getTheta();
	void getDestination();


}Object, ManipulationPoint, WayPoint;


class Motor{
	int forwardPin;
	int reversePin;
	int speedLimit;
	int speedAdjust( int Speed );

public:
	void Speed( int motor_speed );
	void Init( int forward_pin, int reverse_pin, int speed_limit );
}turnMotor, movementMotor;

class Spider
{
  
public:
	byte Address;
	int X;
	int Y;
	double Theta;
	int X_d;
	int Y_d;
	double Theta_d;
	double Alfa;

	void getTheta( double thetaCamera );
	void getAlfaTheta_d();

}Spider;


/**
 * This function is called whenever a wireless packet is received
 */
/*void rfPacketReceived(void)
{
  CCPACKET packet;
  //cc1101.setRxState();
  // Disable wireless reception interrupt
  //detachInterrupt(0);
  if(cc1101.receiveData(&packet) > 0)
  {
    
    if (packet.crc_ok /*&& packet.length > 1*//*)
    {
      // The LED will toggle when a new packet is received
      //Serial.println("R");
      digitalWrite(LEDOUTPUT, !digitalRead(LEDOUTPUT));
      sender_address = packet.data[1];
      X_t = byte_parse( packet.data[2], packet.data[3] ) ;
      Y_t = byte_parse( packet.data[4], packet.data[5] ) ;
      
      Spider.X = byte_parse( packet.data[6], packet.data[7] ) ;
      Spider.Y = byte_parse( packet.data[8], packet.data[9] ) ;
      Spider.getTheta( ( byte_parse(packet.data[10], packet.data[11] ) ) / ( double ) 1000 );

      Object.X = byte_parse( packet.data[12], packet.data[13] );
      Object.Y = byte_parse( packet.data[14], packet.data[15] );
      debug();
     
    }
  }

  // Enable wireless reception interrupt
  //attachInterrupt(0, rfPacketReceived, FALLING);
} */

void cc1101signalsInterrupt()
{
	// set the flag that a package is available
	packet_available = true;
        //Serial.println("PR");
}

void setup()
{
  // Setup LED output pin
  Serial.begin(38400);
  pinMode(LEDOUTPUT, OUTPUT);
  digitalWrite(LEDOUTPUT, LOW);

  //motor init
  movementMotor.Init( 3, 5, 20 );
  turnMotor.Init( 9, 6, 20 );
   
  // Init RF IC
  cc1101.init();
  cc1101.setCarrierFreq(CFREQ_915);//must set frequency to 915MHz, otherwise illegal in US
  cc1101.setChannel(RFCHANNEL, false);
  cc1101.setSyncWord(SYNCWORD1, SYNCWORD0, false);
  cc1101.setDevAddress(SOURCE_ADDR, false);
  
  // Let's disable address check for the current project so that our device
  // will receive packets even not addressed to it.
  //cc1101.disableAddressCheck();
  //Serial.println("S");

  clock = millis();
  attachInterrupt( 0, cc1101signalsInterrupt, FALLING );
}

void loop()
{
  //getCoordinates();
  measure();
  //avoid();
  //rfPacketReceived();
  //if avoid() {
  //movement( X_t, Y_t );
 // }
  //transmit every txTime
  if (((millis() - clock) > (txTime ))){
    transmit(); 
    clock = millis(); 
}

 // delay(1000);                           // Transmit every 5 seconds
  // For low-power applications replace "delay" by "panstamp.sleepWd(WDTO_8S)" for example
}

void getCoordinates()
{
	if( packet_available ){
                CCPACKET packet;
		packet_available = false;//note that the available packet is being read
		detachInterrupt( 0 );
                //Serial.println("packetshere");

		//check for correct packet length
		if( cc1101.receiveData( &packet ) > 14 ){
			if( packet.crc_ok && packet.length > 14 ){
				//Serial.println( "getting data" );
				//Serial.print( "Packet length: " );
				//Serial.println( packet.length );
				//Serial.print( "Data received from: " );
				//Serial.println( packet.data[1] );

				//read in data(the first byte is the receiver address, which is already interpretted through the cc1101 class)
				sender_address = packet.data[1];
				X_t = byte_parse( packet.data[2], packet.data[3] ) ;
				Y_t = byte_parse( packet.data[4], packet.data[5] ) ;

				Spider.X= byte_parse( packet.data[6], packet.data[7] ) ;
				Spider.Y = byte_parse( packet.data[8], packet.data[9] ) ;
				Spider.getTheta( ( byte_parse(packet.data[10], packet.data[11] ) ) / ( double ) 1000 );

				Object.X = byte_parse( packet.data[12], packet.data[13] );
				Object.Y = byte_parse( packet.data[14], packet.data[15] );
                                  //Serial.write("Destination: ");
 /* //Serial.write("X: ");
  Serial.print ( byte_parse(packet.data[10], packet.data[11] ) );
  Serial.print(" ");
  Serial.print (X_t);
  Serial.print(" ");
  //Serial.write("Y: ");
  Serial.print(Y_t);
  //Serial.println();
  Serial.print(" ");
  //Serial.write("Spider: ");
  //Serial.write("X: ");
  Serial.print(Spider.X);
  Serial.print(" ");  
  //Serial.write("Y: ");
  Serial.println(Spider.Y); 
  //Serial.println(" ");*/
				debug();
			}
		}
		attachInterrupt( 0, cc1101signalsInterrupt, FALLING );//reset the interrupt
  }


}


void measure(){
  north = analogRead(N);
  northEast = analogRead(NE);
  east = analogRead(E);
  southEast = analogRead(SE);
  south = analogRead(S);
  southWest = analogRead(SW);
  west = analogRead(W);
  northWest = analogRead(NW);
}

boolean avoid(){
  if (north < 1024) {
    return false;
    //move backwards
  }
  else if (east <  1024) {
    return false;
    //turn left
  }
  else if (south < 1024) {
    return false;
    //move forwards
  }
  else if (west < 1024) {
    return false;
    //turn right
  }
  else {
    return true;
  }
  
    
}


void transmit(){
  //cc1101.setTxState();
  txPacket.length = 17;
  txPacket.data[0] = DESTINATION_ADDR;
  
  byte_conversion(north);
  txPacket.data[1] = LSB;
  txPacket.data[2] = MSB;
  
  byte_conversion(northEast);
  txPacket.data[3] = LSB;
  txPacket.data[4] = MSB;
  
  byte_conversion(east);
  txPacket.data[5] = LSB;
  txPacket.data[6] = MSB;
  
  byte_conversion(southEast);
  txPacket.data[7] = LSB;
  txPacket.data[8] = MSB;

  byte_conversion(south);
  txPacket.data[9] = LSB;
  txPacket.data[10] = MSB;
  
  byte_conversion(southWest);
  txPacket.data[11] = LSB;
  txPacket.data[12] = MSB;
  
  byte_conversion(west);
  txPacket.data[13] = LSB;
  txPacket.data[14] = MSB;
  
  byte_conversion(northWest);
  txPacket.data[15] = LSB;
  txPacket.data[16] = MSB;
  
  cc1101.sendData(txPacket);
  
}

double getDistance( int Dest_x, int Dest_y, int X, int Y )
{
	return sqrt( pow( Dest_x - X, 2 ) + pow( Dest_y - Y, 2 ) );
}

bool movement( int dest_x, int dest_y )
{
	//move this spider to the given destination coordinates

	//return whether the spider has reached the destination or not
	Spider.X_d = dest_x;
	Spider.Y_d = dest_y;
	Spider.getAlfaTheta_d();

	double Rho = getDistance( dest_x, dest_y, Spider.X, Spider.Y );
	
	if( Rho < Mu ){
                movem = 100;
		//stop if near destination
                //Serial.print(Rho);
                //Serial.println("At");
		movementMotor.Speed( 0 );
		turnMotor.Speed( 0 );
		return true;
	}
	else{
		//move to destination
                movem = 200;
		if( cos( Spider.Alfa ) < Sigma ){
			//turning mode
    //                    Serial.print(Rho);
                        //Serial.println("T");
			movementMotor.Speed( 0 );
			if( sin( Spider.Alfa ) >= 0 )
				turnMotor.Speed( TURN_SPEED );
			else
				turnMotor.Speed( -TURN_SPEED );
		}
		else{
    //                    Serial.print(Rho);
                        //Serial.println("M");
                        movem = 300;
                        movementMotor.Speed( SPEED * cos( Spider.Alfa ) * Rho );
			turnMotor.Speed(TURN_SPEED * sin( Spider.Alfa ) );
			// TURN_SPEED * sin( Spider.Alfa ) * cos( Spider.Alfa ) 
		}
	}
	//debug();
        double moveMotor, turnmotor;
        turnmotor = (TURN_SPEED * sin( Spider.Alfa ) );
        moveMotor = ( SPEED * cos( Spider.Alfa ) * Rho );
          //Serial.write("alpha: ");
          //Serial.println(Spider.Alfa);
        //Serial.write("movementMotor.Speed: ");
        //Serial.print(moveMotor);
        //Serial.write("turnMotor.Speed: ");
        //Serial.println(turnmotor);
	return false;
}

void Motor::Speed( int motorSpeed )
{
	//takes a speed from -100 (full reverse) to 100 (full forward)
	//then properly scales it for PWM output

	if( motorSpeed > 5 ){
		motorSpeed = speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                motorSpeed = constrain(motorSpeed,10,30);
		analogWrite( forwardPin, motorSpeed );
		digitalWrite( reversePin, LOW );
	}
	else if( motorSpeed < -5 ){
		motorSpeed = - speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                constrain(motorSpeed,10,30);
		digitalWrite( forwardPin, LOW );
		analogWrite( reversePin, motorSpeed );
	}
	else{
		digitalWrite( forwardPin, LOW );
		digitalWrite( reversePin, LOW );
	}
}

void Motor::Init( int forward_pin, int reverse_pin, int speed_limit )
{
	forwardPin = forward_pin;
	reversePin = reverse_pin;

	pinMode(forwardPin, OUTPUT);
	pinMode(reversePin, OUTPUT);
	speedLimit = speed_limit;
}

int Motor::speedAdjust( int Speed )
{/*
	//sets a lower limit for a motor to prevent stalling
	if( abs( Speed ) < speedLimit ){
		if( Speed > 0 )
			return speedLimit;
		else
			return -speedLimit;
	}
	else return Speed; */ 
        return Speed;
}


void Spider::getAlfaTheta_d() 
{
	//puts theta_d in the range of -pi to pi with 0 in reference to the x-axis
	//calculates alfa
	//do not call unless spider destination is known
        //Serial.write("Theta: ");
        //Serial.println(Theta);

	Theta_d = atan2( Y_d - Y, X_d - X ) ;
	Alfa = Theta_d - Theta;
}

void Spider::getTheta( double thetaCamera )
{
	//puts theta in the range of -pi to pi with 0 in reference to the x-axis

	if( thetaCamera >= 0 && thetaCamera <= pi / 2 )
		Theta = thetaCamera + pi / 2;

	else
		Theta = thetaCamera - 3 * pi / 2;
}


void byte_conversion(int number)
{
	//store least significant byte of number
  LSB = number & 0xff;

    //store  most significant byte of number
  MSB = (number >> 8) & 0xff;
}

int byte_parse( unsigned char LSB, unsigned char MSB )
{
	return ( int ) word( MSB, LSB );
}

void debug()
{	
/*  Serial.print(F("A: "));
  Serial.println(sender_address);
  Serial.print(F( "X: "));
  Serial.println( Spider.X );
  Serial.print(F( "Y: "));
  Serial.println( Spider.Y );
  Serial.print(F( "Theta: "));
  Serial.println( Spider.Theta );
  Serial.print( "Theta_d: ");
  Serial.println( Spider.Theta_d );
  Serial.print(F( "X_t: "));
  Serial.println( X_t );
  Serial.print(F( "Y_t: "));
  Serial.println( Y_t );*/
}

