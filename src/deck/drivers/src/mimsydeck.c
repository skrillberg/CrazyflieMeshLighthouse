#define DEBUG_MODULE "MimsyDeck"
//include "mimsydeck.h"
#include "debug.h"
#include "deck.h"

#include "uart1.h"
#include "stabilizer_types.h"
//#include "commander.h"

#include "FreeRTOS.h"
#include <stdlib.h>
#include "task.h"
#include <stdbool.h>
#include <math.h>
//#include <stdint.h>
#include "system.h"
#include "config.h"
#include "estimator.h"
//#include "openserial.h"
#include "estimator_kalman.h"
#include "estimator.h"

static void handleLighthousePacket(uint8_t * packet);

//function that periodically runs and transmits current state info to mimsy
static void mimsyStateTask(void *param){
	//register current wake time for task
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	//vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ*2); //two second delay
	//set period for mimsy update
	TickType_t rate_hz = 1;
	TickType_t period_ticks = configTICK_RATE_HZ/rate_hz;
	float yaw;
	point_t pos;
	union{
		int16_t val;
		uint8_t bytes[2];
	}x;
	union{
		int16_t val;
		uint8_t bytes[2];
	}y;
	union{
		int32_t val;
		uint8_t bytes[4];
	}phi;
	uint8_t state_packet[10];

	while(estimatorKalmanTest());

	while(1){
		//delay for period of time
		vTaskDelayUntil(&xLastWakeTime, period_ticks);
		//uart1Putchar(uchar);

		//get state information from estimator
		estimatorKalmanGetEstimatedYaw(&yaw);
		estimatorKalmanGetEstimatedPos(&pos);
		//DEBUG_PRINT("yaw: %f, x: %f, y: %f, z: %f \n", yaw, pos.x, pos.y, pos.z);

		//translate state information to bytes
		x.val = (int16_t) (pos.x * 100); //x in centimeters now
		y.val = (int16_t) (pos.y * 100); // y converted to cm
		phi.val = (int32_t) (yaw *3.14159f/180.0f * 1000.0f); //phi converted to milliradians
		//DEBUG_PRINT("yaw: %ld, x: %d, y: %d \n", phi.val, x.val, y.val);

		//construct uart packet
		state_packet[0] = 's';
		state_packet[1] = x.bytes[0];
		state_packet[2] = x. bytes[1];

		state_packet[3] = y.bytes[0];
		state_packet[4] = y.bytes[1];
		state_packet[5] = phi.bytes[0];
		state_packet[6] = phi.bytes[1];
		state_packet[7] = phi.bytes[2];
		state_packet[8] = phi.bytes[3];
		state_packet[9] = 'z';

		//uart1SendDataDmaBlocking(10, state_packet);

		//send uart packet

		for(int i=0; i<10; i++){
			uart1Putchar(state_packet[i]);
		}
	}
}

void updateMimsy(void){



	float yaw;
	point_t pos;
	union{
		int16_t val;
		uint8_t bytes[2];
	}x;
	union{
		int16_t val;
		uint8_t bytes[2];
	}y;
	union{
		int32_t val;
		uint8_t bytes[4];
	}phi;
	uint8_t state_packet[10];


		//get state information from estimator
		estimatorKalmanGetEstimatedYaw(&yaw);
		estimatorKalmanGetEstimatedPos(&pos);
		//DEBUG_PRINT("yaw: %f, x: %f, y: %f, z: %f \n", yaw, pos.x, pos.y, pos.z);

		//translate state information to bytes
		x.val = (int16_t) (pos.x * 100); //x in centimeters now
		y.val = (int16_t) (pos.y * 100); // y converted to cm
		phi.val = (int32_t) (yaw *3.14159f/180.0f * 1000.0f); //phi converted to milliradians
		//DEBUG_PRINT("yaw: %ld, x: %d, y: %d \n", phi.val, x.val, y.val);

		//construct uart packet
		state_packet[0] = 's';
		state_packet[1] = x.bytes[0];
		state_packet[2] = x. bytes[1];

		state_packet[3] = y.bytes[0];
		state_packet[4] = y.bytes[1];
		state_packet[5] = phi.bytes[0];
		state_packet[6] = phi.bytes[1];
		state_packet[7] = phi.bytes[2];
		state_packet[8] = phi.bytes[3];
		state_packet[9] = 'z';

		//uart1SendDataDmaBlocking(10, state_packet);

		//send uart packet

		for(int i=0; i<10; i++){
			uart1Putchar(state_packet[i]);
		}

}

static int parsePacketType(uint8_t preamble){
	enum packet {LIGHTHOUSE,NONE,MAG,DATA,UNKNOWN} packet_type = NONE;
	switch(preamble){
		case 100:
			packet_type = LIGHTHOUSE;
			DEBUG_PRINT("lighthouse packet \n");
			break;
		case 'm':
			packet_type = MAG;
			DEBUG_PRINT("mag packet \n");
			break;

		default:
			packet_type = UNKNOWN;
	}
	return packet_type;

}

static bool processPacket(int type){
	//determine packet length
	enum packet {LIGHTHOUSE,NONE,MAG,DATA,UNKNOWN} packet_type = type;
	char uchar;
	int i = 0;



	int magx;
	int magy;
	int magz;



	uint8_t packet_length;

	//determine packet length based on packet type
	switch(packet_type){
		case LIGHTHOUSE:
			packet_length = 14; //length after preamble, includes EOF char (122)

			break;

		case MAG:
			packet_length = 11;
			break;

		default:
			packet_length = 0;
			break;


	}
	uint8_t packet[packet_length];

	//receive the rest of the packet over uart
	for(i = 0; i < packet_length; i++){

		uart1Getchar(&uchar);
		//uart1Putchar(uchar);
		packet[i] = (uint8_t) uchar;
		//DEBUG_PRINT("Subsequent Char: %i \n",packet[i]);

	}

	//ensure that this packet is actually what it says it is by
	//checking for the end of frame character
	if ((int)uchar != 122){
		DEBUG_PRINT("Checksum Failed, Desynchronizing \n");
		return false;
	}else{
		DEBUG_PRINT("End of Frame: %i \n",(int)uchar);

	}

	//handle packet based on type
	switch(packet_type){

		case LIGHTHOUSE:

			handleLighthousePacket(packet);

			break;

		case MAG:
			magx = 256*packet[2] + packet[3];
			magy = 256*packet[4] + packet[5];
			magz = 256 *packet[6] + packet[7];
			//t = 16777216*packet[8] + 65536*packet[9] + 256*packet[10] + packet[11];
			DEBUG_PRINT("Mag measurement   magx: %d, mag:y %d, magz: %d",(int)magx,(int)magy,(int)magz);
			break;

		default:

			break;
	}

	return true;

}



static mlhMeasurement_t mlh;

static void handleLighthousePacket(uint8_t * packet){

	//test float to see represntation of float
	//3.14 is 0xC3F54840, little endian

	//DEBUG_PRINT("float_test: %f, bytes[0->4]: %x, %x, %x, %x\n",float_test.val, float_test.bytes[0], float_test.bytes[1], float_test.bytes[2], float_test.bytes[3]);

	//union for converting bytes to float, only works if float encoding is the same for crazyflie and mimsy
	/*union{
		float val;
		uint8_t bytes[4];
	} phi;*/

	//unions for converting bytes to ints, only works if endianess is same for crazyflie and mimsy
	union {
		int16_t val;
		uint8_t bytes[2];
	} x;

	union {
		int16_t val;
		uint8_t bytes[2];
	} y;

	union {
		int32_t val;
		uint8_t bytes[4];
	} phi_int;

	float pos_scale = 0.001f; //position scale factor of x, y representation
	//int16_t x =0; //x location of lighthouse in mimsy units
	//int16_t  y=0; //y location of lighthouse in mimsy units
	//float heading_scale = 1.0f; //scale factor of phi representation
	//int32_t phi= 0; //heading measurement in mimsy uints
	uint32_t t=0; //timestamp in ASN time

	/*parse packet for lighthouse measurement.int16_t are little endian on
	* this chip, but current uart interface sends values over as big endian,
	* so the correct bit order needs to be chosen */
	x.bytes[0] = packet[2];
	x.bytes[1] = packet[1];

	y.bytes[0] = packet[4];
	y.bytes[1] = packet[3];

	/*transfer float from packet into unions. Floats are little endian
	 * but serial sends them as big endian at the moment*/
	for(int idx = 0; idx < 4; idx++){
		//phi.bytes[idx] = packet[5+(3-idx)]; //starts packet[8], ends packet[5]
		phi_int.bytes[idx] = packet[5+(3-idx)];
	}


	//convert from 4xbytes to uint32_t
	t = 16777216*packet[9] + 65536*packet[10] + 256*packet[11] + packet[12];



	//TODO: need x and y scale in packet as well to avoid errors and hardcoding
	float wrap = fmodf((float) (phi_int.val) * 0.001f +3.14159f,6.2818f) - 3.14159f;
	DEBUG_PRINT("from mimsy: %f, before mod : %f, after mod: %f, mapped to -pi pi: %f\n",(double) ((float)(phi_int.val) * 0.001f),(double)((float) (phi_int.val) * 0.001f +3.14159f),(double)fmodf((float) (phi_int.val) * 0.001f +3.14159f,6.2818f),(double)wrap);
	mlh.x = (float) (x.val * pos_scale);
	mlh.y = (float) (y.val * pos_scale) ;
	mlh.heading = wrap;
	mlh.measTime = (float) t;


	//send lighthouse measurement to estimator
	estimatorEnqueueMimsyLighthouse(&mlh);

	//debug statements
	/*
	DEBUG_PRINT("x packet: %d, %d\n",packet[1], packet[2] );
	DEBUG_PRINT("y packet: %d, %d\n", packet[3],packet[4]);
	DEBUG_PRINT("phi packet: %d, %d, %d, %d\n", packet[5],packet[6],packet[7],packet[8]);
	DEBUG_PRINT("X bytes: %d, %d\n",x.bytes[0],x.bytes[1]);
	DEBUG_PRINT("Lighthouse Measurement at mimsydeck   x: %d, y: %d, phi: %f, t: %d \n ",x.val,y.val,phi.val,(int)t);
	DEBUG_PRINT("mlh.x: %f, mlh.y %f, mlh.heading: %f, mlh.measTime: %f\n", mlh.x, mlh.y, mlh.heading, mlh.measTime);
	if(success){
		DEBUG_PRINT("Estimator Enqueue Operation: Success \n");
	}else{
		DEBUG_PRINT("Estimator Enqueue Operation: Fail\n");
	}
    */

}

static bool isInit = false;
static void mimsydeckTask(void *param)
{


	//uint8_t c = 'a';
	//setpoint_t setpoint;
	//setpoint.thrust = 0.1;

	bool synchronized = false;

	char pattern[8] = {'d','e','a','d','b','e','e','f'};
	uint8_t synch_num = 0;

//	char str[2] = {'v',0};


	enum packet {LIGHTHOUSE,NONE,MAG,DATA,UNKNOWN} packet_type = NONE;
	systemWaitStart();
	char uchar;
	DEBUG_PRINT("Mimsy Task Begun \n");
	while(!uart1Test()){

	}
	while(1){

		synchronized = true;
		//
		//vTaskDelay(10);
		//DEBUG_PRINT("sup \n");

		//uart1Putchar(uchar);

		//str[0]=uchar;

		//check for "deadbeef" preamble to synchronize with mimsy uart frames
		while(!synchronized){
			uart1Getchar(&uchar);
			//uart1Putchar(uchar);
			DEBUG_PRINT("Char: %i\n",uchar);
			if(uchar == pattern[synch_num]){
				synch_num++;

			}else{
				synch_num = 0;
			}
			if(synch_num == 8){
				synchronized = true;
				DEBUG_PRINT("synchronized! \n");
			}
		}

		//echo uart stream
		uart1Getchar(&uchar);
		//uart1Putchar(uchar);
		DEBUG_PRINT("Char: %i\n",uchar);
		//process packet
		packet_type = parsePacketType((uint8_t)uchar);
		DEBUG_PRINT("Packet Type: %i\n",packet_type);
		synchronized = processPacket(packet_type);

		/*
		if(receiving){
			if((int)uchar == 122){
				DEBUG_PRINT("End of Frame: %i \n",(int)uchar);
				receiving = false;
			}
			DEBUG_PRINT("Subsequent Char: %i \n",(int)uchar);
			uart1Putchar(uchar);

		}
		else{
			uart1Putchar(uchar);
			packet_type = parsePacketType((uint8_t)uchar);
			DEBUG_PRINT("Packet Type: %i\n",packet_type);
			receiving = true;
		}*/


		//commanderSetSetpoint(&setpoint, 5);

	}

}


static void mimsydeckInit()
{
	//systemWaitStart();
	if (isInit) return;
	DEBUG_PRINT("Mimsy Deck Initializing \n");
	uart1Init(115200);

	//commanderInit();
    xTaskCreate(mimsydeckTask, "mimsy", 2*configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(mimsyStateTask, "mimsy state", 2*configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    isInit = true;
}

static bool mimsydeckTest()
{
	DEBUG_PRINT("Mimsy Test passed! \n");
	//DEBUG_PRINT("UART 1 test beginning \n");
	//uint8_t c = 'a';
	//setpoint_t setpoint;
	//setpoint.thrust = 0.5;
	//while(1){
	//uart1Putchar('a');
		//commanderSetSetpoint(&setpoint, 5);
	//}
	return true;
}



static const DeckDriver mimsydeckDriver = {
	.name = "myMimsydeck",
	.init = mimsydeckInit,
	.test = mimsydeckTest,
	.requiredEstimator = kalmanEstimator,
};

DECK_DRIVER(mimsydeckDriver);
