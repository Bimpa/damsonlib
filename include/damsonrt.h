/*
 * SUMMARY
 *  Common symbols for the damson loader and runtimee
 *
 * AUTHORS
 *  Paul Richmond - p.richmond@sheffield.ac.uk
 *
 *
 * COPYRIGHT
 *  Copyright (c) The University of Sheffield, 2013. All rights reserved.
 *  BIMPA Project
 */

#ifndef DAMSON_RUNTIME
#define DAMSON_RUNTIME

/* bool constants */
#define TRUE              			 (0 == 0)
#define FALSE              				(0 != 0)

/* bytes to words conversion */
#define WORDS(x)	    			(x>>2)		//macro to convert bytes to words
#define BYTES(x)				(x<<2)		//macro to convert words to bytes

/* DTCM */
#define ITCM_BASE				0x00000000
#define ITCM_SIZE				0x00008000
#define DTCM_BASE				0x00400000
#define DTCM_SIZE				0x00010000
#define DTCM_TOP				(DTCM_BASE + DTCM_SIZE)

/* SARK C ENVIRONMENT */
#define SARK_DTCM_RESERVED			0x00002000	//4kb reserved memory for C constants

/* DAMSON stacks - stack sizes must be power of 2! */
#define DAMSONRT_D_STACK			(8192*4)					//32kb of damson stack space in bytes (GV[7]*4)
#define DAMSONRT_I_STACK			(512*4)						//2kb of interrupt stack
#define DAMSONRT_C_STACK			(4096*4)					//16kb of c stack

#define DAMSONRT_TOTAL_STACKS			(DAMSONRT_D_STACK + DAMSONRT_I_STACK + DAMSONRT_C_STACK)	//50kb
#define DAMSONRT_C_STACK_START			(DTCM_TOP-DAMSONRT_C_STACK)
#define DAMSONRT_I_STACK_START			(DAMSONRT_C_STACK_START-DAMSONRT_I_STACK)
#define DAMSONRT_D_STACK_START			(DAMSONRT_I_STACK_START-DAMSONRT_D_STACK)

#define DAMSONRT_SINGLE_PROCESS_D_STACK		(128*4)							//damson stack size in bytes of single process
#define DAMSONRT_MAX_PROCESSES			(DAMSONRT_D_STACK/DAMSONRT_SINGLE_PROCESS_D_STACK)	//maximum number of damson processes
#define DAMSONRT_SINGLE_PROCESS_C_STACK		(DAMSONRT_C_STACK/DAMSONRT_MAX_PROCESSES)		//process C stack size in bytes

/* DAMSON DTCM*/
#define DAMSONRT_DTCM_START 			(DTCM_BASE+SARK_DTCM_RESERVED)							// start of damson dtcm
#define DAMSONRT_DTCM_DATA_MAX   		(DTCM_SIZE-SARK_DTCM_RESERVED-DAMSONRT_TOTAL_STACKS) 	// 10kb space for damson globals
#define DAMSONRT_DTCM_PROGRAM_START		(DAMSONRT_DTCM_START+DAMSONRT_DTCM_DATA_MAX)

/* DAMSON globals*/
#define DAMSONRT_SYSTEM_RESERVED		(50*4)							// GV[0]*4
#define DAMSONRT_SYSTEM_GLOBAL_ADDRESS(n)	((unsigned int*)(DAMSONRT_DTCM_START+(n*4)))		// damson GV address at word n
#define DAMSONRT_SYSTEM_GLOBAL(n)   		(*((unsigned int*) DAMSONRT_SYSTEM_GLOBAL_ADDRESS(n)))	// damson GV value at word n

/* SDRAM */
#define DAMSONRT_EV_SIZE	  		(7*1024*1024) 			// bytes of EV  per core (assumes 16 cores 1-16 usable) = 7 megabytes per core
#define DAMSONRT_EV_START(n)   			(0x70000000 +((n-1)*DAMSONRT_EV_SIZE))	// starting address of ev for a core number 0<n<=16
#define DAMSONRT_EV_SHARED_SIZE			(16*1024*1024) 			// size (bytes) of SDRAM shared space (16 Megabytes)
#define DAMSONRT_EV_SHARED_START 		DAMSONRT_EV_START(17) 	// start address of SDRAM shared

/* clock rate*/
#define DAMSONRT_TICKRATE_HZ			1		//1 second tick

/* DAMSON port */
#define	DAMSONRT_PORT_BITS			11
#define DAMSON_PORT_MASK			((1<<DAMSONRT_PORT_BITS)-1)

/* logging */
#define MAX_LOG_ITEMS				5		//Max number of items to be logged

/* hash constants */
#define DAMSONRT_HASH_A 			137		// A value prime for runtime hash function
#define DAMSONRT_HASH_C 			92731		// C value prime for runtime hash function

/* debug output */
#define DAMSONLIB_DEBUG 			TRUE		//debug output for damsonlib
#define API_PRINT_DLY 				200		//usec delay to ensure io has completed


#define MAX_ROUTING_TABLE_ENTRIES		999



#endif //DAMSON_RUNTIME

