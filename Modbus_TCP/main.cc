/*
 * Copyright Â© 2008-2010 StÃ©phane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <modbus.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;


// Access from ARM Running Linux
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdlib.h>
#include <fcntl.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

// Note: Revision2.0 RaspberryPis, GPIO Pins are: 2, 3, 4, 7, 8, 9, 10, 11, 14, 15, 17, 18, 22, 23, 24, 25, 27, with 28, 29, 30, 31 additionally available on the P5 header)
// for pinout info see http://elinux.org/RPi_Low-level_peripherals#GPIO_Driving_Example_.28C.29

// define GPIO number for each pin (using rev2 of pi)
// T= Top row of pins
// B= Bottom row of pins
#define T_8 14
#define T_10 15
#define T_12 18
#define T_16 23
#define T_18 24
#define T_22 25
#define T_24 8
#define T_26 7

#define B_3 2
#define B_5 3
#define B_7 4
#define B_11 17
#define B_13 27
#define B_15 22
#define B_19 10
#define B_21 9
#define B_23 11

enum {
    TCP,
    RTU
};

int updateVirtualMeterValue(int currentReading){
	
		// Calculate a new value to be sent in next reply
		// Introduce First Order Growth hear
		// currentReading = currentReading * exp();
		currentReading = currentReading + 10;
		return currentReading;
}

// Calculates the average current value to write
float calcCurrentAvg(modbus_mapping_t *mb_mapping, float currentAvg){
	
	// Define a union for holding uint and float variables
	union floatToInt {
		float current;
		unsigned int value;
	} flt;

	// Determine the new current Average value
	float minimum = currentAvg - 5;
	float maximum  = currentAvg + 5;
	float randomSeed = ((float) rand()) / (float) (RAND_MAX);
	float difference = maximum - minimum;
	float multiplier = randomSeed * difference;
	float current = minimum + multiplier;

	// Check if we hit any boundary values
	if (current < 0){
		current = 0;
	}

	if (current > 63){
		current = 63;
	}

	// Store calculated values into union components
	flt.current = current;
	uint32_t i;
	i = flt.value;

	// Assign the variables to registers
	mb_mapping->tab_registers[3000] = (uint16_t)i;
	mb_mapping->tab_registers[2999] = (uint16_t)(i >> 16)&0xffff;

	// Return a local value
	return current;
}

// Calculates the Voltage value to write
float calcVoltageLN(modbus_mapping_t *mb_mapping){
	
	// Define a union for holding uint and float variables
	union floatToInt {
		float voltage;
		unsigned int value;
	} flt;

	// Determine a voltage value to be added
	float randomSeed = ((float) rand()) / (float) (RAND_MAX);
	float difference = 245-235;
	float multiplier = randomSeed * difference;
	float voltage = 235 + multiplier;

	// Store calculated values into union components
	flt.voltage = voltage;
	uint32_t i;
	i = flt.value;

	// Write to the mapped registers
	mb_mapping->tab_registers[3028] = (uint16_t)i;
	mb_mapping->tab_registers[3027] = (uint16_t)(i >> 16)&0xffff;

	// Return a local value
	return voltage;
}

// Calculates the real power total to write
float calcRealPowerTotal(modbus_mapping_t *mb_mapping, float voltageLN, float currentAvg){
	
	// Define a union for holding uint and float variables
	union floatToInt {
		float power;
		unsigned int value;
	} flt;
	
	// Calculate the real power total
	float realPower = (voltageLN * currentAvg) / 1000;
	
	// Store calculated values into union components
	flt.power = realPower;
	uint32_t i;
	i = flt.value;
	
	// Write to the mapped registers
	mb_mapping->tab_registers[3054] = (uint16_t)i;
	mb_mapping->tab_registers[3053] = (uint16_t)(i >> 16)&0xffff;

	// Return a local value
	return realPower;
}

// Calculates the reactive power total to write
float calcReactivePowerTotal(modbus_mapping_t *mb_mapping, float realPower, float powerFactor){

	// Define a union for holding uint and float variables
	union floatToInt {
		float power;
		unsigned int value;
	} flt;
	
	// Calculate the reactivePower
	float reactivePower = realPower * sin(powerFactor);
	
	// Store calculated values into union components
	flt.power = realPower;
	uint32_t i;
	i = flt.value;
	
	// Write to the mapped registers
	mb_mapping->tab_registers[3068] = (uint16_t)i;
	mb_mapping->tab_registers[3067] = (uint16_t)(i >> 16)&0xffff;
	
	// Return a local value
	return reactivePower;
}

// Calculates the apparent power total to write
float calcApparentPowerTotal(modbus_mapping_t *mb_mapping, float realPower, float reactivePower){

	// Define a union for holding uint and float variables
	union floatToInt {
		float power;
		unsigned int value;
	} flt;
	
	// Determine the apparentPowerTotal
	float apparentPower = sqrt((realPower * realPower) + (reactivePower * reactivePower));
	
	// Store calculated values into union components
	flt.power = realPower;
	uint32_t i;
	i = flt.value;
	
	// Write to mapped registers
	mb_mapping->tab_registers[3076] = (uint16_t)i;
	mb_mapping->tab_registers[3075] = (uint16_t)(i >> 16)&0xffff;
	
	// Return a local value
	return apparentPower;
}

// Calculates the power factor total to write
float calcPowerFactor(modbus_mapping_t *mb_mapping){
	// Float between .8 & 1
	union floatToInt {
		float powerFactor;
		unsigned int value;
	} flt;

	// Determine a voltage value to be added
	float randomSeed = ((float) rand()) / (float) (RAND_MAX);
	float difference = 1-0.8;
	float multiplier = randomSeed * difference;
	float powerFactor = 0.8 + multiplier;
	
	// Store calculated values into union components
	flt.powerFactor = powerFactor;
	uint32_t i;
	i = flt.value;
	
	// Write to the mapped registers
	mb_mapping->tab_registers[3084] = (uint16_t)i;
	mb_mapping->tab_registers[3083] = (uint16_t)(i >> 16)&0xffff;

	// Return a local value
	return powerFactor;
}

float calcFrequency(modbus_mapping_t *mb_mapping){
	// Use random generation to select a value between 48 and 52
	// Define a union for holding uint and float variables
	union floatToInt {
		float frequency;
		unsigned int value;
	} flt;

	// Determine a voltage value to be added
	float randomSeed = ((float) rand()) / (float) (RAND_MAX);
	float difference = 52-48;
	float multiplier = randomSeed * difference;
	float frequency = 48 + multiplier;
	
	// Store calculated values into union components
	flt.frequency = frequency;
	uint32_t i;
	i = flt.value;
	
	// Write to the mapped registers
	mb_mapping->tab_registers[3110] = (uint16_t)i;
	mb_mapping->tab_registers[3109] = (uint16_t)(i >> 16)&0xffff;
	
	// Return a local value
	return frequency;
}

// Calculates the real energy into the load to write
uint64_t calcRealEnergyIn(modbus_mapping_t *mb_mapping, float realPower, int timeInterval){
	
	// Calculate Wh
	float calc = (realPower * 1000) / (3600/timeInterval);
	
	uint16_t tempVariable;
	
	// Begin register check
	if ((mb_mapping->tab_registers[3206] + calc) < 65535){
		// Write to register
		mb_mapping->tab_registers[3206] += calc;
	}
	else{
		// Value is bigger than register
		tempVariable = 65535 - mb_mapping->tab_registers[3206];
		
		mb_mapping->tab_registers[3206] = calc - tempVariable;
		// Check for overflow on second register
		if (mb_mapping->tab_registers[3205] < 65535) {
			// Add to second register
			mb_mapping->tab_registers[3205] += 1;
		}
		
		else {
			// Set second register to 0
			mb_mapping->tab_registers[3205] = 0;

			// Check for overflow on third register
			if (mb_mapping->tab_registers[3204] < 65535) {
				// Add to third register
				mb_mapping->tab_registers[3204] += 1;
			}
			
			else {
				// Set third register to 0
				mb_mapping->tab_registers[3204] = 0;
				
				// Check for overflow on fourth register
				if (mb_mapping->tab_registers[3203] < 65535) {
					// Add to fourth register
					mb_mapping->tab_registers[3203] += 1;
				}
				else {
					// Set fourther register to 0 to prevent memory overflow
					mb_mapping->tab_registers[3203] = 0;
				}
				
			}
		}
	}

	return 0;
}

// Calculates the reactive energy into the load to write
uint64_t calcReactiveEnergyIn(modbus_mapping_t *mb_mapping, float reactivePower, int timeInterval){
	
		// Calculate Wh
	float calc = (reactivePower * 1000) / (3600/timeInterval);
	
	uint16_t tempVariable;
	
	// Begin register check
	if ((mb_mapping->tab_registers[3222] + calc) < 65535){
		// Write to register
		mb_mapping->tab_registers[3222] += calc;
	}
	else{
		// Value is bigger than register
		tempVariable = 65535 - mb_mapping->tab_registers[3222];
		
		mb_mapping->tab_registers[3222] = calc - tempVariable;
		
		// Check for overflow on seocond register
		if (mb_mapping->tab_registers[3221] < 65535) {
			// Add to second register
			mb_mapping->tab_registers[3221] += 1;
		}
		
		else {
			// Set second register to 0
			mb_mapping->tab_registers[3221] = 0;
			
			// Check for overflow on third register
			if (mb_mapping->tab_registers[3220] < 65535) {
				// Add to third register
				mb_mapping->tab_registers[3220] += 1;
			}
			
			else {
				// Set third register to 0
				mb_mapping->tab_registers[3220] = 0;
				
				// Check for overflow on fourth register
				if (mb_mapping->tab_registers[3219] < 65535) {
					// Add to fourth register
					mb_mapping->tab_registers[3219] += 1;
				}
				else {
					// Set fourth register to 0 to prevent overflow of memory
					mb_mapping->tab_registers[3219] = 0;
				}
				
			}
		}
	}

	return 0;
}

int main(int argc, char **argv)
{
	// Local Declarations
    int socket;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    int rc;
    int use_backend;
    int nPort = 502; // default for single instance
	int i;
    time_t startIntervalTime = time(0);
	time_t currentIntervalTime;
	bool initialTransaction(true);
	bool updateStatus(false);
	int slaveID;
	int newID;
	int timeInterval = 10;
	
	// Calculation Variables
	float currentAvg = 0;
	float voltageLN = 240;
	float realPower = 0;
	float reactivePower = 0;
	float apparentPower = 0;
	float frequency = 48;
	float powerFactor = 0.8;
	uint64_t realEnergyIn = 0;
	uint64_t realEnergyOut = 0;
	uint64_t reactiveEnergyIn = 0;
	uint64_t reactiveEnergyOut = 0;
	uint64_t activeEnergyA = 0;
	uint64_t activeEnergyB = 0;
	
	// Define a union for holding uint and float variables
	union floatToInt {
		float floatValue;
		unsigned int intValue;
	} flt;
	uint32_t storeInt;
	
	// Introduction Message
	cout << "Virtual Modbus Meter Executable" << endl << endl;
	
	// Perform check on input variable
//	int suppliedNode;
//	cout << "Please provide the node address for the device: ";
//	cin >> suppliedNode;
//	cout << endl << endl;
//	
//	if (suppliedNode >= 1 && suppliedNode <= 254)
//	{
//		// Perform the desired task
//		printf("We were supplied the valid node ID: %i \r\n", suppliedNode);
//	}
//	else {
//		// Wait until we have a valid connection
//		printf("We were supplied the invalid node ID: %i \r\n", suppliedNode);
//	}

    // to emulate a large MODBUS device we need at least 10000 input and holding registers
    mb_mapping = modbus_mapping_new(1000, 1000, 10000, 10000);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
	
	// Enable TCP communication and await a connection
    use_backend = TCP;
	printf("Modbus Server:\nRunning in tcp mode - Modbus client for testing a server\n\n");
	printf("Waiting for TCP connection on Port %i \n\n",nPort);
	ctx = modbus_new_tcp("127.0.0.1", nPort);
	socket = modbus_tcp_listen(ctx, 1);
	int error = modbus_tcp_accept(ctx, &socket);

	// Enter main loop - writing to registers / replying to requests
    for(;;) {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

        rc = modbus_receive(ctx, query);
		
		// Verify node address matches
//		slaveID = query[6];
//		while (slaveID != suppliedNode){
//			printf("Node ID provided does not match slave device ID.\n");
//			modbus_close(ctx); // close
//			sleep(1000*5); // Sleep for 5 seconds before allowing a new connection attempt
//			modbus_tcp_accept(ctx, &socket); // Allow for a new attempt to connect
//			rc = modbus_receive(ctx, query); // Allow an initial message containing slaveID
//			slaveID = query[6];
//		}
		
        if (rc >= 0) {
	    int nToShow = 10;
	    int i=0,nCard=0;
		
		// Output request sent from software
	    printf("Replying to request num bytes=%i ",rc);
		printf("\nQuery sent: ");
	    for(i=0;i<rc;i++)
	      printf("%i, ",query[i]);
	    printf("\n");
	      
            modbus_reply(ctx, query, rc, mb_mapping);
	    // Output reply sent to software
	    printf("Holding Registers = ");
	    for( i=0;i<nToShow;i++)
		printf("%i, ",mb_mapping->tab_registers[i]);
	    printf("\n");
		printf("\n");
		
		// Check to see if valid amount of time has passed for an update
		currentIntervalTime = time(0);
		if (difftime(currentIntervalTime, startIntervalTime) >= timeInterval){
			updateStatus = true;
			startIntervalTime = currentIntervalTime;
		}
	    
		if (initialTransaction){
			mb_mapping->tab_registers[0]++;		 // increment the holding reg 0 for each read
			mb_mapping->tab_registers[2013] = 1; // Number of Phases
			mb_mapping->tab_registers[2014] = 2; // Number of Wires
			
			// Current Average (default to 30A)
			mb_mapping->tab_registers[2999] = 0;
			mb_mapping->tab_registers[3000] = 0;
			mb_mapping->tab_registers[3001] = 0;
			mb_mapping->tab_registers[3002] = 0;
			
			// Voltage L-N
			mb_mapping->tab_registers[3027] = 0;
			mb_mapping->tab_registers[3028] = 0;
			mb_mapping->tab_registers[3029] = 0;
			mb_mapping->tab_registers[3030] = 0;
			
			// Real Power Total
			mb_mapping->tab_registers[3053] = 0; 
			mb_mapping->tab_registers[3054] = 0;
			mb_mapping->tab_registers[3055] = 0;
			mb_mapping->tab_registers[3056] = 0;
			
			// Reactive Power Total
			mb_mapping->tab_registers[3067] = 0; 
			mb_mapping->tab_registers[3068] = 0;
			mb_mapping->tab_registers[3069] = 0;
			mb_mapping->tab_registers[3070] = 0;
			
			// Apparent Power Total
			mb_mapping->tab_registers[3075] = 0; 
			mb_mapping->tab_registers[3076] = 0;
			mb_mapping->tab_registers[3077] = 0;
			mb_mapping->tab_registers[3078] = 0;
			
			// Power Factor Total
			mb_mapping->tab_registers[3083] = 0; 
			mb_mapping->tab_registers[3084] = 0;
			mb_mapping->tab_registers[3085] = 0;
			mb_mapping->tab_registers[3086] = 0;
			
			// Frequency
			mb_mapping->tab_registers[3109] = 0; 
			mb_mapping->tab_registers[3110] = 0;
			mb_mapping->tab_registers[3111] = 0;
			mb_mapping->tab_registers[3112] = 0;
			
			// Open the text file stream, retrieve the last stored values from previous metering session
			ifstream data;
			data.open("StoredValues.dat");
			
			// Real Energy Into the Load
			data >> mb_mapping->tab_registers[3203]; 
			data >> mb_mapping->tab_registers[3204];
			data >> mb_mapping->tab_registers[3205];
			data >> mb_mapping->tab_registers[3206];
			
			// Real Energy Out of the Load
			mb_mapping->tab_registers[3207] = 0; 
			mb_mapping->tab_registers[3208] = 0;
			mb_mapping->tab_registers[3209] = 0;
			mb_mapping->tab_registers[3210] = 0;
			
			// Reactive Energy Into the Load
			data >> mb_mapping->tab_registers[3219]; 
			data >> mb_mapping->tab_registers[3220];
			data >> mb_mapping->tab_registers[3221];
			data >> mb_mapping->tab_registers[3222];
			
			// Close the stream
			data.close();
			
			// Reactive Energy out of the Load
			mb_mapping->tab_registers[3223] = 0; 
			mb_mapping->tab_registers[3224] = 0;
			mb_mapping->tab_registers[3225] = 0;
			mb_mapping->tab_registers[3226] = 0;
			
			// Active Energy Delivered A
			mb_mapping->tab_registers[4195] = 0; 
			mb_mapping->tab_registers[4196] = 0; 
			mb_mapping->tab_registers[4197] = 0; 
			mb_mapping->tab_registers[4198] = 0; 
			
			// Active Energy Delivered B
			mb_mapping->tab_registers[4199] = 0; 
			mb_mapping->tab_registers[4200] = 0;
			mb_mapping->tab_registers[4201] = 0;
			mb_mapping->tab_registers[4202] = 0;
			initialTransaction = false;
		}
		else if (updateStatus){
	    // every time we do a communication, update a bunch of the registers so we have something interesting to plot on the graphs
			for (// each id)
            mb_mapping->tab_registers[0]++;														// increment the holding reg 0 for each read
			mb_mapping->tab_registers[2014] = 1;												// Number of Phases
			mb_mapping->tab_registers[2015] = 2;												// Number of Wires
			currentAvg = calcCurrentAvg(mb_mapping, currentAvg);								// Current Average
			voltageLN = calcVoltageLN(mb_mapping);												// Voltage L-N
			realPower = calcRealPowerTotal(mb_mapping, voltageLN, currentAvg);					// Real Power Total
			reactivePower = calcReactivePowerTotal(mb_mapping, realPower, powerFactor);			// Reactive Power Total
			apparentPower = calcApparentPowerTotal(mb_mapping, realPower, reactivePower);		// Apparent Power Total
			powerFactor = calcPowerFactor(mb_mapping);											// Power Factor
			frequency = calcFrequency(mb_mapping);												// Frequency
			realEnergyIn = calcRealEnergyIn(mb_mapping, realPower, timeInterval);				// Real Energy Into Load
			reactiveEnergyIn = calcReactiveEnergyIn(mb_mapping, reactivePower, timeInterval);	//Reactive Energy Into Load
			
			updateStatus = false;
		} else {
			// Check connection
			
		}
        } else {
            /* Connection closed by the client or server */
            printf("Con Closed.\n");
			modbus_close(ctx); // close
			// immediately start waiting for another request again
            modbus_tcp_accept(ctx, &socket);
        }
		
		// Write the newest values to the text file for storage / usage
		ofstream data;
		data.open("StoredValues.dat", ios::trunc);
		
		data << mb_mapping->tab_registers[3203] << endl; 
		data << mb_mapping->tab_registers[3204] << endl;
		data << mb_mapping->tab_registers[3205] << endl;
		data << mb_mapping->tab_registers[3206] << endl;
		
		data << mb_mapping->tab_registers[3219] << endl; 
		data << mb_mapping->tab_registers[3220] << endl;
		data << mb_mapping->tab_registers[3221] << endl;
		data << mb_mapping->tab_registers[3222] << endl;
		
		data.close();
    }

    printf("Quit the loop: %s\n", modbus_strerror(errno));

    modbus_mapping_free(mb_mapping);
    close(socket);
    modbus_free(ctx);

    return 0;
}
