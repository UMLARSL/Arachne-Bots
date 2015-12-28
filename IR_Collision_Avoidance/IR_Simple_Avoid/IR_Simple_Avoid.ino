//This program uses a movement algorithm that will be implemented in
//waypoint-based applications such as object manipulation, object avoidance, and cooperative control.
//This program is meant to be used with the panstamp microcontroller, and associated library.
//Authors: Jack Price, Damien Laird, Edited by Ben Sullivan

//libraries needed for radio communication
#include <EEPROM.h>
#include <cc1101.h>

//constants
const double pi = 4 * atan( ( double ) 1 );
#define Sigma 0.3 //range of error for theta_rg
#define Mu 30 //range of error for rho
#define SPEED 1
#define TURN_SPEED 30
#define AVOID 25
#define TURNAVOID 10

//Variables
int X_t, Y_t ;//coordinates of final destination
bool send_state = false;

//LED Pins
#define NORTHPIN A0
#define NORTHEASTPIN A1
#define EASTPIN A2
#define SOUTHEASTPIN A3
#define SOUTHPIN A4
#define SOUTHWESTPIN A5
#define WESTPIN A6
#define NORTHWESTPIN A7
#define EMITTER 8
int north, northeast, east, southeast, south, southwest, west, northwest;
int highest, direc;

//radio information
CC1101 cc1101;//radio class
CCPACKET packet;//data packet
byte sender_address;//sender of received transmission
byte spider_address = 7;//unique radio address, different for each spider
byte network_address[] = {19, 9};//identifies the network configuration, must be the same for all spiders
boolean packet_available = false;
boolean at_destination;

class Motor{
	int forwardPin;
	int reversePin;
	int speedLimit;
	int speedAdjust( int Speed );

public:
	void moveSpeed( int motor_speed );
        void turnSpeed( int motor_Speed );
	void Init( int forward_pin, int reverse_pin, int speed_limit );
}turnMotor, movementMotor;

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


void setup()
{
	//serial init
	Serial.begin( 38400 );
	Serial.println( "Start" );

	//motor init
	movementMotor.Init( 9, 6, 20 );
	turnMotor.Init( 5, 3, 20 );

	//radio init
	cc1101.init();
	cc1101.setCarrierFreq( CFREQ_915 );//FREQUENCY MUST BE SET TO 915MHZ, OTHERWISE RADIO OPERATION IS ILLEGAL IN US
	cc1101.setSyncWord( network_address, false );
	cc1101.setDevAddress( spider_address, false );
	cc1101.enableAddressCheck();
	cc1101.setRxState();
	attachInterrupt( 0, cc1101signalsInterrupt, FALLING );
	Serial.print( "Spider Address:" );
	Serial.println( spider_address );

        
        pinMode(EMITTER,HIGH);
        digitalWrite(EMITTER,HIGH);
        at_destination = false;
        
	Serial.println( "Initialized" );
}

void loop()
{
  getCoordinates();  
  Measure();
  CheckHighest();
  avoid();
  //movement( X_t, Y_t );
 
}

void Measure(){
  int nRead,neRead,eRead,seRead,sRead,swRead,wRead,nwRead;
  nRead = analogRead(NORTHPIN);
  neRead = analogRead(NORTHEASTPIN);
  eRead = analogRead(EASTPIN);
  seRead = analogRead(SOUTHEASTPIN);
  sRead = analogRead(SOUTHPIN);
  swRead = analogRead(SOUTHWESTPIN);
  wRead = analogRead(WESTPIN);
  nwRead = analogRead(NORTHWESTPIN);
  
  north = GetDistance(nRead);
  northeast = GetDistance(neRead);
  east = GetDistance(eRead);
  southeast = GetDistance(seRead);
  south = GetDistance (sRead);
  southwest = GetDistance(swRead);
  west = GetDistance(wRead);
  northwest = GetDistance(nwRead);
}

int GetDistance(int reading){
  
  if(reading < 45){
    return 5; }
  else if (reading < 450){
    return 10; }
  else if (reading < 700) {
    return 15; }
  else if (reading < 810) {
    return 20; }
  else if (reading < 865) {
    return 25; }
  else if (reading < 1020) {
    return 30; }
  else{
    return 0; }
    
}

//loops through direction readings and finds the highest
void CheckHighest(){
  int directions [8] = {north, northeast, east, southeast, south, southwest, west, northwest};
  highest = 0;
  for (int i = 0; i < 8; i++){
    if (directions[i] > highest){
      highest = directions[i];
      direc = i;
    }
  }
  if (direc == 0){
    north = highest;
    northeast = east = southeast = south = southwest = west = northwest = 0;
  }
  else if( direc == 1){
    northeast = highest;
    north = east = southeast = south = southwest = west = northwest = 0;
  }
  else if( direc == 2 ){
    east = highest;
    north = northeast = southeast = south = southwest = west = northwest = 0;
  }
  else if( direc == 3){
    southeast = highest;
    north = northeast = southeast = south = southwest = west = northwest = 0;
  }
  else if( direc == 4){
    south = highest;
    north = northeast = east = south = southwest = west = northwest = 0;
  }
  else if( direc == 5){
    southwest = highest;
    north = northeast = east = southeast = south = west = northwest = 0;
  }
  else if( direc == 6){
    west = highest;
    north = northeast = east = southeast = south = southwest = northwest = 0;
  }
  else if( direc == 7){
    northwest = highest;
    north = northeast = east = southeast = south = southwest = west = 0;
  }
  
}


void avoid(){
  if (north > 0){
    if (not at_destination){
      movementMotor.moveSpeed( 0 );
      turnMotor.turnSpeed (TURNAVOID);
      movementMotor.moveSpeed(0);
      delay(1500);
      turnMotor.turnSpeed(0);
      movementMotor.moveSpeed(AVOID);
      delay(1500);
    }
  }
  else if (northeast > 0){
    movementMotor.moveSpeed(AVOID);
    turnMotor.turnSpeed(-TURNAVOID);
  }
 else if (east > 0){
  movementMotor.moveSpeed( 0 );
  turnMotor.turnSpeed (-TURNAVOID);
 } 
 else if (southeast > 0){
   movementMotor.moveSpeed( -AVOID );
   turnMotor.turnSpeed (-TURNAVOID);
 }
 else if (south > 0){
   movementMotor.moveSpeed( AVOID );
   turnMotor.turnSpeed (0);
 }
 else if (southwest > 0){
   movementMotor.moveSpeed( -AVOID );
   turnMotor.turnSpeed (TURNAVOID);
 }
 else if (west > 0){
   movementMotor.moveSpeed( 0 );
   turnMotor.turnSpeed (TURNAVOID);
 }
 else if (northwest > 0){
   movementMotor.moveSpeed( AVOID );
   turnMotor.turnSpeed (TURNAVOID);
 }
 else {
   movement( X_t, Y_t );
 }
}


//Radio and coordinate functions
void cc1101signalsInterrupt()
{
	// set the flag that a package is available
	packet_available = true;
}

int byte_parse( unsigned char LSB, unsigned char MSB )
{
	return ( int ) word( MSB, LSB );
}

void getCoordinates()
{
	if( packet_available ){
		packet_available = false;//note that the available packet is being read
		detachInterrupt( 0 );
                //Serial.println("packetshere");

		//check for correct packet length
		if( cc1101.receiveData( &packet ) > 14 ){
			if( packet.crc_ok && packet.length > 14 ){
				Serial.println( "getting data" );
				Serial.print( "Packet length: " );
				Serial.println( packet.length );
				Serial.print( "Data received from: " );
				Serial.println( packet.data[1] );

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
  //Serial.write("X: ");
 /* Serial.print ( byte_parse(packet.data[10], packet.data[11] ) );
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
		//stop if near destination
                //Serial.print(Rho);
                Serial.println("At Destination");
		movementMotor.moveSpeed( 0 );
		turnMotor.turnSpeed( 0 );
                at_destination = true;
		return true;
	}
	else{
    at_destination = false;
		//move to destination
		if( cos( Spider.Alfa ) < Sigma ){
			//turning mode
    //                    Serial.print(Rho);
                        Serial.println("Turning");
			movementMotor.moveSpeed( 0 );
			if( sin( Spider.Alfa ) >= 0 )
				turnMotor.turnSpeed( TURN_SPEED );
			else
				turnMotor.turnSpeed( -TURN_SPEED );
		}
		else{
    //                    Serial.print(Rho);
                        Serial.println("Turning and Moving");
                        movementMotor.moveSpeed( SPEED * cos( Spider.Alfa ) * Rho );
			turnMotor.turnSpeed(TURN_SPEED * sin( Spider.Alfa ) );
			// TURN_SPEED * sin( Spider.Alfa ) * cos( Spider.Alfa ) 
		}
	}
	debug();
        double moveMotor, turnmotor;
        turnmotor = ((TURN_SPEED * sin( Spider.Alfa ) ));
        moveMotor = (( SPEED * cos( Spider.Alfa ) * Rho ));
          //Serial.write("alpha: ");
          //Serial.println(Spider.Alfa);
        //Serial.write("movementMotor.Speed: ");
        //Serial.print(moveMotor);
        //Serial.write("turnMotor.Speed: ");
        //Serial.println(turnmotor);
	return false;
}


void Motor::moveSpeed( int motorSpeed )
{
	//takes a speed from -100 (full reverse) to 100 (full forward)
	//then properly scales it for PWM output

	if( motorSpeed > 5 ){
		motorSpeed = speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                motorSpeed = constrain(motorSpeed,20,40);
		analogWrite( forwardPin, motorSpeed );
		digitalWrite( reversePin, LOW );
	}
	else if( motorSpeed < -5 ){
		motorSpeed = - speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                constrain(motorSpeed,20,40);
		digitalWrite( forwardPin, LOW );
		analogWrite( reversePin, motorSpeed );
	}
	else{
		digitalWrite( forwardPin, LOW );
		digitalWrite( reversePin, LOW );
	}
}

void Motor::turnSpeed( int motorSpeed )
{
	//takes a speed from -100 (full reverse) to 100 (full forward)
	//then properly scales it for PWM output

	if( motorSpeed > 5 ){
		motorSpeed = speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                motorSpeed = constrain(motorSpeed,10,25);
		analogWrite( forwardPin, motorSpeed );
		digitalWrite( reversePin, LOW );
	}
	else if( motorSpeed < -5 ){
		motorSpeed = - speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
                constrain(motorSpeed,10,25);
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

void debug()
{/*	Serial.print( "X: ");
	Serial.println( Spider.X );
	Serial.print( "Y: ");
	Serial.println( Spider.Y );
	Serial.print( "Theta: ");
	Serial.println( Spider.Theta );
	Serial.print( "Theta_d: ");
	Serial.println( Spider.Theta_d );
	Serial.print( "X_t: ");
	Serial.println( X_t );
	Serial.print( "Y_t: ");
	Serial.println( Y_t );*/
}
