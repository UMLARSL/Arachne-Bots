//This program sends receives spider coordinate packets from serial
//then sends the packet to the spider with the corresponding address
//used for object manipulation data

//libraries needed for radio communication
#include <EEPROM.h>
#include <cc1101.h>

//radio information
byte network_address[] = {19, 9};//must  be same for all panstamps[
#define SENDER_ADDRESS 4//address of panstamp attached to computer
CC1101 cc1101;//radio class
CCPACKET packet;//data packet

//variables for checking correct data transmission
unsigned char LSB;
unsigned char MSB;

//functions
int byte_parse(unsigned char LSB, unsigned char MSB );//parse bytes used for transmission into data integers
void byte_conversion(int number);//change integers into two bytes to transfer on radio packet
void read_coordinates();//read serial coordinate data being sent from computer
void send_coordinates();//send coordinate data via radio

void setup()
{
	//serial init
	Serial.begin(38400);
	Serial.println("Start");

	//radio init
	cc1101.init();
	packet.length = 16;//number of data bytes transmitted
	cc1101.setCarrierFreq(CFREQ_915);//must set frequency to 915MHz, otherwise illegal in US
	cc1101.setSyncWord(network_address, false);//set network address
	cc1101.setDevAddress(SENDER_ADDRESS, false);//set device address
	Serial.print("Sender Address: ");
	Serial.println(SENDER_ADDRESS);

	Serial.println("Initizalized");
}

void loop()
{
	read_coordinates();
}

void read_coordinates()
{
	//check for correct number of bytes
	if(Serial.available() ){
		//check for start delimiter to packet
		Serial.findUntil("-./", "-./");
		packet.data[0] = (byte)Serial.parseInt();//receiver address
		packet.data[1] = SENDER_ADDRESS;

		byte_conversion(Serial.parseInt());//x_r
		packet.data[2] = LSB;//stores least significant byte of x_r into packet
		packet.data[3] = MSB;//stores most significant byte of x_r into packet
			
		byte_conversion(Serial.parseInt());//y_r
		packet.data[4] = LSB;
		packet.data[5] = MSB;

		byte_conversion(Serial.parseInt());//x
		packet.data[6] = LSB;
		packet.data[7] = MSB;

		byte_conversion(Serial.parseInt());//y
		packet.data[8] = LSB;
		packet.data[9] = MSB;

		byte_conversion(Serial.parseInt());//phi
		packet.data[10] = LSB;
		packet.data[11] = MSB;

		byte_conversion(Serial.parseInt());//x_o
		packet.data[12] = LSB;
		packet.data[13] = MSB;

		byte_conversion(Serial.parseInt());//y_o
		packet.data[14] = LSB;
		packet.data[15] = MSB;

		if( Serial.findUntil("+++", "+++")){
			send_coordinates();	
		}

		Serial.flush();
	}
}

void send_coordinates()
{
	//send data and report if it was properly sent and to which radio
	if(cc1101.sendData(packet)){
		//Serial.print("Sent to: ");
		//Serial.println(packet.data[0]);	
	}
	else{
		Serial.print("Failed to send to: ");
		Serial.println(packet.data[0]);
	}
}

void byte_conversion(int number)
{
	//store least significant byte of number
    LSB = number & 0xff;

    //store  most significant byte of number
    MSB = (number >> 8) & 0xff;
}

int byte_parse(unsigned char LSB, unsigned char MSB )
{
	return (int)word(MSB, LSB);
}
