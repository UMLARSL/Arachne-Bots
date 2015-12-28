//This program uses a movement algorithm that will be implemented in
//waypoint-based applications such as object manipulation, object avoidance, and cooperative control.
//This program is meant to be used with the panstamp microcontroller, and associated library.
//Authors: Jack Price, Damien Laird

//libraries needed for radio communication
#include <EEPROM.h>
#include <cc1101.h>

//constants
const double pi = 4 * atan( ( double ) 1 );
#define Sigma 0.9 //range of error for theta_rg
#define Mu 15 //range of error for rho
#define SPEED 127
#define TURN_SPEED 127

//Variables
int X_t, Y_t ;//coordinates of final destination
bool send_state = false;

//radio information
CC1101 cc1101;//radio class
CCPACKET packet;//data packet
byte sender_address;//sender of received transmission
byte spider_address = 6;//unique radio address, different for each spider
byte network_address[] = {19, 9};//identifies the network configuration, must be the same for all spiders
boolean packet_available = false;

class Motor{
	int forwardPin;
	int reversePin;
	int speedLimit;
	int speedAdjust( int Speed );

public:
	void Speed( int motor_speed );
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
	movementMotor.Init( 3, 5, 20 );
	turnMotor.Init( 9, 6, 20 );

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

	Serial.println( "Initialized" );
}

void loop()
{
	getCoordinates();

	movement( X_t, Y_t );
	
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
		movementMotor.Speed( 0 );
		turnMotor.Speed( 0 );
		return true;
	}
	else{
		//move to destination
		if( cos( Spider.Alfa ) < Sigma ){
			//turning mode
			movementMotor.Speed( 0 );
			if( sin( Spider.Alfa ) >= 0 )
				turnMotor.Speed( TURN_SPEED );
			else
				turnMotor.Speed( -TURN_SPEED );
		}
		else{
			movementMotor.Speed( SPEED * cos( Spider.Alfa ) * Rho );
			turnMotor.Speed(TURN_SPEED * sin( Spider.Alfa ) );
			// TURN_SPEED * sin( Spider.Alfa ) * cos( Spider.Alfa ) 
		}
	}
	debug();

	return false;
}


void Motor::Speed( int motorSpeed )
{
	//takes a speed from -100 (full reverse) to 100 (full forward)
	//then properly scales it for PWM output

	if( motorSpeed > 5 ){
		motorSpeed = speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
		analogWrite( forwardPin, motorSpeed );
		digitalWrite( reversePin, LOW );
	}
	else if( motorSpeed < -5 ){
		motorSpeed = - speedAdjust( ( int )( ( double ) motorSpeed * 1.27 ) );
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
{
	//sets a lower limit for a motor to prevent stalling
	if( abs( Speed ) < speedLimit ){
		if( Speed > 0 )
			return speedLimit;
		else
			return -speedLimit;
	}
	else return Speed;
}


void Spider::getAlfaTheta_d() 
{
	//puts theta_d in the range of -pi to pi with 0 in reference to the x-axis
	//calculates alfa
	//do not call unless spider destination is known

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
{
	Serial.print( "X: ");
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
	Serial.println( Y_t );
}