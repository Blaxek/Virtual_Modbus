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


// Access from ARM Running Linux
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>

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


//
// Set up a memory regions to access GPIO (must be called before trying to access IO!)
//
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem , will run without GPIO Control\n");
      return;
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;


} // setup_io


enum {
    TCP,
    RTU
};

int main(int argc, char *argv[])
{
    int socket;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    int rc;
    int use_backend;
    int nPort = 1500; // default for single instance

    setup_io(); // prepare the IO to be accessed!
    
    // to emulate a large MODBUS device we need at least 10000 input and holding registers
    mb_mapping = modbus_mapping_new(1000, 1000, 10000, 10000);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    
    /* RTU */
    use_backend = RTU;

	printf("Waiting for Serial connection on /dev/ttyUSB0\n");
        ctx = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
        modbus_set_slave(ctx, 1); // Sets slave ID to 1, can be changed to be a specified slave ID from 1 - 254 as in the TCP implementation
        modbus_connect(ctx); // Connect up to an external program e.g. Modscan, PME
	printf("Serial connection started!\n");

    for(;;) {
        uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

        rc = modbus_receive(ctx, query);
        if (rc >= 0) {
	    int nToShow = 10;
	    int i=0,nCard=0;

	    printf("Replying to request num bytes=%i (",rc);
	    for(i=0;i<rc;i++)
	      printf("%i, ",query[i]);
	    printf(")\n");
	      
            modbus_reply(ctx, query, rc, mb_mapping);
	    
	    // After each communication, show ModBus registers so you can see what is happening
	    printf("Holding Registers (tab_registers) = ");
	    for( i=0;i<nToShow;i++)
		printf("%i, ",mb_mapping->tab_registers[i]);
	    printf("\n");
	    
	    // every time we do a communication, update a bunch of the registers so we have something interesting to plot on the graphs
            mb_mapping->tab_registers[0]++; // increment the holding reg 0 for each read
            mb_mapping->tab_registers[1] = rand(); // this register is a full scale random number 0 - 0xffff
   	    mb_mapping->tab_input_registers[0] = 2; // version number
            for( i=1;i<nToShow;i++)
	    {
	        // randomly increase or decrease the register, but do not allow wrapping
		if( rand() > RAND_MAX/2 )
		{		
		    if ( mb_mapping->tab_input_registers[i] < 0xfffe );
			mb_mapping->tab_input_registers[i] += 1;

		    if( mb_mapping->tab_registers[i+1] < 0xfffe )
			mb_mapping->tab_registers[i+1] += 1;
		}
		else
		{
		    if( mb_mapping->tab_input_registers[i] > 0 )
		        mb_mapping->tab_input_registers[i] -= 1;
		    if( mb_mapping->tab_registers[i+1] > 0 )
		        mb_mapping->tab_registers[i+1] -= 1;
		} 
	    }
        } else {
            /* Connection closed by the client or server */
            printf("Con Closed.\n");
	    modbus_close(ctx); // close
	    // immediately start waiting for another request again
            modbus_tcp_accept(ctx, &socket);
        }
    }

    printf("Quit the loop: %s\n", modbus_strerror(errno));

    modbus_mapping_free(mb_mapping);
    close(socket);
    modbus_free(ctx);

    return 0;
}