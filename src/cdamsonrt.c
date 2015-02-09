/*
 * SUMMARY
 *  C functions for the damsonlib runtime system
 *
 * AUTHORS
 *  Paul Richmond - p.richmond@sheffield.ac.uk
 *
 *
 * COPYRIGHT
 *  Copyright (c) The University of Sheffield, 2013. All rights reserved.
 *  BIMPA Project
 */
 
#include "spinnaker.h"
#include "spinn_io.h"
#include "damsonrt.h"

/* important SpiNNaker and SARK defines*/
#define EXCEPTION_BASE 				0x00000020
#define RX_READY_PRIORITY     			3	//VIC priorities from spin1_api_params.h
#define DMA_DONE_PRIORITY      			2	//VIC priorities from spin1_api_params.h
#define TIMER1_PRIORITY        			1	//VIC priorities from spin1_api_params.h
#define CC_TMT_PRIORITY        			4	//VIC priorities from spin1_api_params.h
#define SYS_MC_ENTRIES 				24
#define APP_MC_ENTRIES 				(MC_TABLE_SIZE - SYS_MC_ENTRIES)
#define CORE_ROUTE(core)     			(1 << (core + NUM_LINKS))





/* DMA */
#define DMA_BURST_SIZE        4
#define DMA_WIDTH             1
#define DMA_READ              0
#define DMA_WRITE             1

/* DMA Usage Mode */
#define CPU_DMA_STATUS_FREE		0
#define CPU_DMA_STATUS_DAMSON_TRANSFER	1
#define CPU_DMA_STATUS_LOGGING_TRANSFER	2

/* Log buffering */
#define LOG_BUFFER_STATUS_FREE 		0
#define LOG_BUFFER_STATUS_OCCUPIED	1
#define MAX_LOG_BUFFERS			5
#define LOGGING_WITH_DMA		1		// remove definition to log without using DMA controller - this is slower!


/* PCB status */
#define PCB_STATUS_FREE				0
#define PCB_STATUS_RUNNING			1
#define PCB_STATUS_DELAYING			2
#define PCB_STATUS_DMATRANSFER			3
#define PCB_STATUS_DMAWAITING			4

/* Stack overflow detetcion */
#define STACK_END_VALUE 			0xDEADBEEF


/* DMA transfer request item */
typedef struct
{
	void		*system_address;
	void 		*local_address;
	uint		description;
} DMAWaitItem;


/* Log bugger item (local storage for a single log entry)*/
typedef struct
{
	short int status;
	DMAWaitItem dma;
	uint data[MAX_LOG_ITEMS+2]; //extra 2 items for handle and item count
}LogBufferItem;

/* PCB struct */
typedef struct
{
	uint           	*saved_FP;
	uint 		*saved_SP;
	uint           	*saved_LR;
	uint           	*saved_PC;
	uint           	*saved_PSR;
	uint           	saved_r0;
	uint           	saved_r1;
	uint           	saved_r2;
	uint           	saved_r3;
	uint		saved_r4;
	uint		saved_r5;
	uint		saved_r6;
	uint		saved_r7;
	uint		saved_r8;
	uint		saved_r9;
	uint 		saved_r10;
	uint		saved_r12;
	uint		*stack;
	uint		*c_stack;
	short int	handle;
	char		status;
	char 		priority;
	DMAWaitItem	dma;
	int		delay_us;
} PCB;

/* log item (describes a log or snapshot) - structure also defined in loader.c */
typedef struct
{
	short int 	handle;
	uint		start_time;	//us
	uint 		end_time;	//us
	uint		interval;	//us
	int		interval_count;	//us (used to count down)
	uint 		log_items;
	int*		log_globals[MAX_LOG_ITEMS];
} LogItem;

/* interrupt vector item */
typedef struct
{
	uint  src_node;
	uint  code_offset;
	uint  count;
} InterruptVectorItem;


/* routing table entry */
typedef struct {
	unsigned int key; 	//damson source node id
	unsigned int route;     //the route bits, [0-5]=links, [6-23]=cores
}RoutingEntry;


/* a few sark externals */
extern uint virt_cpu;
extern uint phys_cpu;
extern uchar rootChip; 
extern void vic_init (void);
extern void restore_sark(void);

/* externals from damson code generator object and linker */
extern void _mainprog(void);
extern uint DAMSON_OBJ_BASE;	//start address of DAMSON code generated module
extern uint SARK_DATA_END;

/* externals from sync module */
extern void sync_init(uint c, uint *cm);
extern void sync_barrier_wait(void);

/* externals from DAMSONRT assembly module */
extern int udiv(uint d, uint n);
extern void init_damsonlib(void);
extern void restore_process(PCB *pcb); 
extern void end_process(void); 
extern void timer_interrupt(void); 
extern void packet_interrupt(void);
extern void syncnodes_packet_interrupt(void);
extern void dma_interrupt();
extern void enable_interrupts(void); 
extern void disable_interrupts(void); 
extern void sleep(void);

/* prototypes referenced by assembly module */
extern void c_main();
extern uint c_create_process(void *pc, char pr, uint arg0, uint arg1, uint arg2, uint arg3);
extern void c_delete_process(uint h);
extern void c_reschedule();
extern void c_tickrate (uint time);
extern void c_sendpkt (uint src_id, uint port, uint data);
extern void c_damsonexit(unsigned int exit_val);
extern void c_syncnodes();
extern void c_handle_timer();
extern void c_handle_packet(uint key);
extern void c_handle_sdtransfer(void *system_address, void *local_address, uint words, uint direction);
extern void c_handle_sdcomplete();
extern void c_handle_delay(int fp_seconds);

/* internal prototypes */
void init_interrupts();
void init_dma_controller();
void init_routing_table();
void init_stack();
void init_overflow_detection();
void init_interrupt_vector();
void init_logging();
void log_entry(LogItem* log);
void process_log_buffers();
uint hash(uint n, uint size);
InterruptVectorItem *get_interrupt(uint i);
void clear_mc_tables();
void set_mc_table_entry(uint entry, uint key, uint mask, uint route);
void delay_us (uint us);
uint mod(uint a, uint b);
int round_delay(int delay, uint tickrate);


/* routing tables */
static volatile uint * const rtr_mcrte = (uint *) RTR_MCRAM_BASE;
static volatile uint * const rtr_mckey = (uint *) RTR_MCKEY_BASE;
static volatile uint * const rtr_mcmsk = (uint *) RTR_MCMASK_BASE;

/* VIC vector tables */
typedef void (*isr_t) (void);
volatile isr_t * const vic_vectors  = (isr_t *) (VIC_BASE + 0x100);
static volatile uint * const vic_controls = (uint *) (VIC_BASE + 0x200);
static isr_t * const exception_table = (isr_t *) EXCEPTION_BASE;

/* internal globals exported for use in assembler*/
PCB* 			CURRENT_PCB;			 	// currently active PCB slot
void*			I_STACK_END;
void*			DAMSON_GV_START;
uint			NODE_ID;

/* internal runtime globals */
uchar 			rootAp = 0;				// flag for root processor
uint 			chips;					// number of chips in board layout
uint 			ticks = 0;		   		// clock tick count
uint			simtime_us = 0;				// the simulation time in microseconds
uint			tickrate = 0;				// clock tickrate in microseconds
uint			last_schedule_ticks = 0;		// tickcount at last re-schedule
PCB 			p_list[DAMSONRT_MAX_PROCESSES]; 	// simple linear list of PCB slots
LogBufferItem 		log_buffers[MAX_LOG_BUFFERS];		// local buffer storage for DMA controller
short int		cpu_dma_status;				// DMA use mode either NONE, DAMSON or LOGGING
short int		dma_handle;				// PCB handle or Log entry item handle

InterruptVectorItem 	*intv;					// pointer to interrupt vector (set by loader)
InterruptVectorItem 	*timerint;				// pointer to interrupt entry for timer
short int		intvitems;				// size of the interrupt vector (number of hash entries)
uint 			*extern_start;				// the end address of the external vector
uint 			*extern_end;				// the end address of the external vector
uint 			start_time_ms;				// start time of execution
uint 			end_time_ms;				// end time of execution
LogItem			*logs;					// the logs 
LogItem			*snapshots;				// the snapshots 
short int		num_logs;				// the number of logs
short int		num_snapshots;				// the number of snapshots
uint 			*log_entry_item;			// current decrementing address to store log entries 
char			debug_mode;				// debug mode for the current node

uint active_processes = 0;
uint created_processes = 0;


/** @brief Main entry function for any damson program linked with damsonlib
  * This is the main entry function for any damson program linked with damsonlib. It is responsible 
  * for initialisation of synchronisation code, the interrupt vectors, the process control block stacks, 
  * the DAMSON runtime reserved vector and initialising interrupts. The root application processor (first 
  * to be loaded on the chip) also initialises the routing tables. A new processes is created for the damson 
  * main entry function. A global synchronisation is performed and the damson main entry function starts by 
  * calling reschedule.
  */
void c_main ()
{
	uint *core_map;
	uint i;
	uint h;
		
	if ((uint)&SARK_DATA_END > DAMSONRT_DTCM_START){
		io_printf (IO_STD, "ERROR: Runtime C data size %d exceeds maximum %d\n", (uint)&SARK_DATA_END-DTCM_BASE, SARK_DTCM_RESERVED);
		delay_us(API_PRINT_DLY);
		return;
	}
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "\t[dlib] C data size=%d, maximum=%d\n", (uint)&SARK_DATA_END-DTCM_BASE, SARK_DTCM_RESERVED);
	}
	#endif
		
	
	//set damson gv start global
	DAMSON_GV_START = (void*)DAMSONRT_DTCM_START;
	
	//get from damson gv
	NODE_ID = DAMSONRT_SYSTEM_GLOBAL(49);
	chips = DAMSONRT_SYSTEM_GLOBAL(48);	
	core_map = (uint*) DAMSONRT_EV_SHARED_START;
	
	
	//debug mode
	debug_mode = (char)DAMSONRT_SYSTEM_GLOBAL(24);
	
	
	//is this the root application processor (previous used locking but not necessary)
	rootAp = 0;
	if (virt_cpu == 1)
		rootAp = TRUE;		
	
	
	//init
	if (rootAp){
		clear_mc_tables();
		init_routing_table();
		#if DAMSONLIB_DEBUG == TRUE
		if (debug_mode){
			io_printf (IO_STD, "\t[dib] monitor pcpu is %d\n", sv->v2p_map[0]);
			delay_us(API_PRINT_DLY);
		}
		#endif
	}

	init_dma_controller();
	init_interrupt_vector();
	init_stack();
	init_interrupts(); //start interrupts
	init_logging();
	init_damsonlib();  //assembly module (enables interrupts)
			
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "\t[dib] Node %d (vcpu %d, pcpu %d) alive!\n", NODE_ID, virt_cpu, phys_cpu);
		delay_us(API_PRINT_DLY);
	}
	#endif
	
	//create a new PCB for main with no arguments
	c_create_process(_mainprog, 0, 0, 0, 0, 0);

	//sync before start
	sync_init(chips, core_map);
	//c_syncnodes();
	
	//enable interrupts only for sync
	enable_interrupts();
	sync_barrier_wait();
	disable_interrupts();
	
	//delay to make sure all cores have disabled interrupts before rescheduling
	delay_us(500000);

	//set tickrate and start the timer
	c_tickrate(DAMSONRT_TICKRATE_HZ); 
	
	//get the start time of execution and set dma status
	start_time_ms = (sv->time_sec*1000)+sv->time_ms;
	cpu_dma_status = 0;
	
	//init stack checking;
	init_overflow_detection();
	
	//reschedule
	c_reschedule();
}

/** @brief Creates a new process in a free PCB slot
  * Performs a linear search to find the first free PCB (indicated by having a FP of 0). When a
  * free slot is found the PCB stack is initialised by adding a frame for end_process function. 
  * This is required as after completing any damson code the PCB associated with the process must 
  * be made available. Any arguments for the new processes are saved into the PCB values. The 
  * saved FP and LR are set to the end_process frame and end_process function respectively. The 
  * PCBs saved PC is set to the pc argument address. 
  *
  *
  * @param pc		A void pointer of the program address for the new process
  * @param arg0		A uint value for the first argument to be passed to the new process
  * @param arg1		A uint value for the second argument to be passed to the new process
  * @param arg2		A uint value for the third argument to be passed to the new process
  * @param arg3		A uint value for the fourth argument to be passed to the new process
  *
  * @return 		A process handle (the uint index of the PCB in the p_list)
  */              
uint c_create_process(void *pc, char pr, uint arg0, uint arg1, uint arg2, uint arg3)
{
	short int i;
	
	
	//find a free PCB slot
	for (i=0; i< DAMSONRT_MAX_PROCESSES; i++)
	{
		//SP == 0 indicates empty slot
		if (p_list[i].status == PCB_STATUS_FREE)
		{
			active_processes++;
			created_processes++;
		
			//create a damson frame at top stack for end_process
			p_list[i].stack[0] = 0; 			//no more frames
			p_list[i].stack[1] = 0;				//lr empty
			p_list[i].stack[2] = (uint)end_process;		//pc is start of procedure (not required)
			p_list[i].stack[3] = i;				//handle for process
			//init saved state for restore process
			p_list[i].saved_FP = p_list[i].stack;			//fp is start of end_process frame
			p_list[i].saved_SP = p_list[i].c_stack;			//reset the C stack pointer
			p_list[i].saved_LR = (uint*)end_process;		//lr is end_process
			p_list[i].saved_PC = pc;				//pc is pc of new processes
			p_list[i].saved_PSR = (uint*)0x1f;			//no flags set for psr
			p_list[i].saved_r0 = arg0;				//argument on frame
			p_list[i].saved_r1 = arg1;				//argument on frame
			p_list[i].saved_r2 = arg2;				//argument on frame
			p_list[i].saved_r3 = arg3;				//argument on frame
			p_list[i].saved_r4 = (uint)(p_list[i].saved_FP + 4);	//r4 is temp frame pointer (offset by size of end_process frame)
			p_list[i].saved_r5 = 0;					//r5 is empty
			p_list[i].saved_r6 = 0;					//r6 is empty
			p_list[i].saved_r7 = 0;					//r7 is empty
			p_list[i].saved_r8 = 0;					//r8 is empty
			p_list[i].saved_r9 = 0;					//r9 is empty
			p_list[i].saved_r10 = DAMSONRT_DTCM_START;		//gv
			p_list[i].saved_r12 = (uint)&p_list[i];			//r12 is pointer to this pcb
			p_list[i].status = PCB_STATUS_RUNNING;
			p_list[i].priority = pr;
			p_list[i].dma.system_address = 0;
			p_list[i].dma.local_address = 0;
			p_list[i].dma.description = 0;
			p_list[i].delay_us = 0;
			
						
			#if DAMSONLIB_DEBUG == TRUE
			if (debug_mode){
				io_printf (IO_STD, "\t[dib] new process(%d): PC=0x%x a=%d, c=%d, tks=%d\n", i, pc, active_processes, created_processes, ticks);
				delay_us(API_PRINT_DLY);
			}
			#endif
			return i;
		}
	}
		
	io_printf (IO_STD, "Error: No more processes left!\n");
	return 0;
}

/** @brief Frees a PCB slot given the handle
  * This function is called by end_process after the handle h has been retrieved from the 
  * damson stack frame.The handle is an index into the PCB list. It is used to mark the PCB 
  * as free by setting the FP to 0. All other process values are reset when the PCB is recycled by
  * creating a new process.
  *
  * @param h		A handle (index in p_list) for the PCB to be removed 
  */
void c_delete_process(uint h)
{
	uint * c_stack_end;
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "\t[dib] deleting process: %d, c=%d, tks=%d\n", h, created_processes, ticks);
		delay_us(API_PRINT_DLY);
	}
	#endif
	
	active_processes--;
	
	//set status indicate it is a free PCB slot
	p_list[h].status = PCB_STATUS_FREE;
	
	//check for damson stack overflows caused by this process
	if (p_list[h].stack[WORDS(DAMSONRT_SINGLE_PROCESS_D_STACK)-1] != STACK_END_VALUE)
	{
		io_printf (IO_STD, "\tERROR: Stack overflow caused by process: %d\n", h);
		delay_us(API_PRINT_DLY);
		p_list[h].stack[WORDS(DAMSONRT_SINGLE_PROCESS_D_STACK)-1] = STACK_END_VALUE;
	}
	
	//check for C stack overflows caused by this process
	c_stack_end = ((uint*)DAMSONRT_C_STACK_START + ((h)*WORDS(DAMSONRT_SINGLE_PROCESS_C_STACK)));
	if (*c_stack_end != STACK_END_VALUE)
	{
		io_printf (IO_STD, "\tERROR: C Stack overflow caused by process: %d\n", h);
		delay_us(API_PRINT_DLY);
		*c_stack_end = STACK_END_VALUE;
	}
	
	
	c_reschedule();
}

/** @brief A basic scheduler
  * This reschedule function iterates the list of PCBs and selects the running thread to be restored. 
  * If a DMA waiting thread is found and the DMA controller is free then the DMA request is handled 
  * (without restoring the thread). If there has been an advance in the number of clock ticks since 
  * the last reschedule then any delay threads are updated to reduce the delay value by the clock 
  * ticks in microseconds. If the delay has completed then the PCB status is updated to running.
  *
  * Once the last running slot is found the process is restarted  (or started for the first time) by 
  * calling restore_process with a pointer to the PCB. The PCB must contain the saved states of 
  * the registers for the process. The CURRENT_PCB pointer is used to indicate that there is an 
  * active process. 
  *
  */
void c_reschedule(){
	short int i;
	char current_pcb_pr;
	
	CURRENT_PCB = NULL;
	current_pcb_pr = 0;
			
	//find first process and execute it
	for (i=DAMSONRT_MAX_PROCESSES-1; i>=0; i--)
	{
		//warn if unserviced PCBs (only occurs on clock interrupt) - ignore the PCB which is the timer interrupt
		//#if DAMSONLIB_DEBUG == TRUE
		//if (debug_mode){
		if (ticks>last_schedule_ticks)
		{
			if((p_list[i].status == PCB_STATUS_RUNNING)&&(p_list[i].saved_PC != (uint*)timerint->code_offset)){
				io_printf (IO_STD, "\t[dib] WARNING: PCB=%d not serviced! act=%d, tks=%d, ltks=%d\n", i, active_processes, ticks, last_schedule_ticks);
				delay_us(API_PRINT_DLY);
			}
		}
		//}
		//#endif
		
		//always serve a wait DMA request first if possible (if DMA controller is free and not waiting for completion)
		if ((p_list[i].status == PCB_STATUS_DMAWAITING) && (cpu_dma_status == CPU_DMA_STATUS_FREE) && (!(dma[DMA_STAT] & 4)))
		{
			dma[DMA_ADRS] = (uint) p_list[i].dma.system_address;
			dma[DMA_ADRT] = (uint) p_list[i].dma.local_address;
			dma[DMA_DESC] = p_list[i].dma.description;
			p_list[i].status = PCB_STATUS_DMATRANSFER;
			cpu_dma_status = CPU_DMA_STATUS_DAMSON_TRANSFER;
			dma_handle = i;
		}
		
		if ((p_list[i].status == PCB_STATUS_DELAYING)&&(ticks>last_schedule_ticks))
		{
			p_list[i].delay_us -= tickrate;
			p_list[i].delay_us = round_delay(p_list[i].delay_us, tickrate);	//rounding
			if (p_list[i].delay_us == 0)
				p_list[i].status = PCB_STATUS_RUNNING;
		}
		
		//TODO: semaphore
		
		//test to see if its a running process with higher priority than any previous
		if (p_list[i].status == PCB_STATUS_RUNNING)
		{
			if(p_list[i].priority >= current_pcb_pr){
				CURRENT_PCB = &p_list[i];
				current_pcb_pr = p_list[i].priority;
			}
		}
	}
	
	//increment ticks
	if (ticks>last_schedule_ticks)
		last_schedule_ticks = ticks;
	
	//restore
	if (CURRENT_PCB != NULL){
		#if DAMSONLIB_DEBUG == TRUE
		if (debug_mode){
			io_printf (IO_STD, "\t[dib] scheduler restoring PCB=%d\n", CURRENT_PCB->handle);
			delay_us(API_PRINT_DLY);
		}
		#endif
		restore_process(CURRENT_PCB);
	}
	
	//enter sleep state
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "Nothing to do. Sleeping.....\n");
		delay_us(API_PRINT_DLY);
	}
	#endif
	
	//go back to sys mode and sleep
	sleep();

	io_printf (IO_STD, "ERROR: If you can see this then sleeping failed!\n");
	delay_us(API_PRINT_DLY);
}


/** @brief A function for setting the timer interrupt
  * Sets the timer interrupt for the processor. The default interrupt is 1Hz.
  * 
  * Unclear if the timer can be set independently for each core or chip!
  *
  * @param freq		A uint frequency (Hz) for the tickrate
  */
void c_tickrate (uint freq)
{
	tc[T1_CONTROL] = 0;	// Disable
	tc[T1_INT_CLR] = 1;	// Clear interrupts

	uint time_us;
	time_us = udiv(freq, 1000000);;	

	if (time_us)
	{
		#if DAMSONLIB_DEBUG == TRUE
		if (debug_mode){
			io_printf(IO_STD, "\t[dib] tickrate set to %d micoseconds\n", time_us);
			delay_us(API_PRINT_DLY);
		}
		#endif
		tickrate = time_us;
		tc[T1_LOAD]    = sv->cpu_clk * time_us;
		tc[T1_BG_LOAD] = sv->cpu_clk * time_us;
		tc[T1_CONTROL] = 0xe2;  // 32-bit, periodic, ints enabled
	}
}

/** @brief A function for sending a multi cast packet
  * This function sends a multicast packet. If the comms controller hardware buffer and 
  * the software buffer are full then the function blocks until it becomes free.
  *
  * The SpiNNaker API quest mc packet sends and uses the VIC to handle dequeuing. We could 
  * adopt this approach in the future.
  *
  * @param src_id		A uint value of the damson sorce id
  * @param port			A uint value of the damson port
  * @param data			A uint value of the packet payload
  */
void c_sendpkt (uint src_id, uint port, uint data)
{
	uint key;
	uint i;
	

	//update any snapshot logs
	for (i=0;i<num_snapshots;i++)
	{
		//log if within time bounds
		if ((snapshots[i].start_time <= simtime_us)&&(snapshots[i].end_time > simtime_us))
		{
			log_entry(&snapshots[i]);
		}
	}

	//send when possible
	while ((cc[CC_TCR] & BIT_31) == 0)
	{
		continue;
	}
	key = (src_id << DAMSONRT_PORT_BITS) + port;	//shift by the number of bits used for the port	
	cc[CC_TCR] = TCR_MC;
	cc[CC_TXDATA] = data;
	cc[CC_TXKEY] = key;

	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] sendpkt: key=%d, payload=%d\n", key, data);
		delay_us(API_PRINT_DLY);
	}
	#endif
}


/** @brief A function damson node exit
  * Exit syncronises between cores by disabling the timer interrupt and setting the mc packet handle to 
  * a special function for handling only the sync packets (i.e. not damson packets). The position of the 
  * log_entry_item write buffer is saved to the start of external SDRAM to be used by the loader. If the core
  * is the root core of the root chip it signals to the loader that execution is complete. All cores then 
  * restore the SARK environment and enter sleep mode.
  *
  *
  * @param exit_Val		A uint exit value
  */
void c_damsonexit(unsigned int exit_val)
{
	//set DAMSON global 25 to end address of log entries
	*DAMSONRT_SYSTEM_GLOBAL_ADDRESS(25) = (uint)log_entry_item;
	
	io_printf(IO_STD, "HOSTCMD:exit %d\n", exit_val);
	delay_us (API_PRINT_DLY);
		
	//sync
	vic[VIC_DISABLE] = (1 << TIMER1_INT) | (1 << CC_RDY_INT);
	exception_table[7] = syncnodes_packet_interrupt;
	vic_vectors[RX_READY_PRIORITY]  = syncnodes_packet_interrupt;
	vic[VIC_ENABLE] = (1 << CC_RDY_INT);
	sync_barrier_wait();
	
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t\t[dib] exit_sync\n");
		delay_us (API_PRINT_DLY);
	}
	#endif
	
	//if core root
	if ((rootAp) && (rootChip))
	{
		end_time_ms = (sv->time_sec*1000)+sv->time_ms;
		delay_us (1000);		//wait 1 ms for all cores to exit
		//send special host cmd message
		io_printf(IO_STD, "HOSTCMD:ticks %d\n", ticks);
		delay_us (API_PRINT_DLY);
		io_printf(IO_STD, "HOSTCMD:shutdown %d\n", end_time_ms-start_time_ms);
		delay_us (API_PRINT_DLY);
	}
	
	//restore SARK environment
	dma[DMA_GCTL] = 0;    // disable all IRQ sources
	dma[DMA_CTRL] = 0x3f; // Abort pending and active transfers
	dma[DMA_CTRL] = 0x0d; // clear possible transfer done and restart
	//vic_init();
	restore_sark();
	
}

/** @brief Global sync function
  * Untested!
  */
void c_syncnodes(){
		
	// set sync_packet_interrupt as FIQ (ignores damson packets) and disable timer interrupts
	vic[VIC_DISABLE] = (1 << TIMER1_INT) | (1 << CC_RDY_INT);
	exception_table[7] = syncnodes_packet_interrupt;
	vic_vectors[RX_READY_PRIORITY]  = syncnodes_packet_interrupt;
	vic[VIC_ENABLE] = (1 << CC_RDY_INT);
	
	//sync
	sync_barrier_wait();
	
	// set  normal packet_interrupt as FIQ and re-enable timer interrupts (if appropriate)
	vic[VIC_DISABLE] = (1 << TIMER1_INT) | (1 << CC_RDY_INT);
	exception_table[7] = packet_interrupt;
	vic_vectors[RX_READY_PRIORITY]  = packet_interrupt;
	vic[VIC_ENABLE] = (1 << TIMER1_INT) | (1 << CC_RDY_INT);
}

/** @brief Handles a timer interrupt once the state has been saved
  * 
  */
void c_handle_timer(){
	uint cycles;
	uint i;
		
	tc[T1_INT_CLR] = 1;				//clear the timer interrupt
	ticks++;					//increment the tick count
	simtime_us += tickrate;				//increment the system time

	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] timer %d PC is 0x%x\n", ticks, timerint->code_offset);
		delay_us(API_PRINT_DLY);
	}
	#endif
	
	//update any timer logs
	for (i=0;i<num_logs;i++)
	{
		//decrease to microsecond interval
		logs[i].interval_count -= tickrate;
		
		//log if within time bounds
		if (logs[i].interval_count <= 0)
		{
			logs[i].interval_count = tickrate;
			if ((logs[i].start_time < simtime_us)&&(logs[i].end_time >= simtime_us))
			{
				log_entry(&logs[i]);
			}
		}
	}
	
	//if there is a clock interrupt then create a new process
	if (timerint->code_offset != 0)
	{	
		cycles = tc[T1_LOAD]*ticks;			//number of cycles per clock * number of clock ticks
		for (i=0; i<timerint->count ; i++){
			c_create_process((void*)timerint->code_offset, 3, 0, 0, 0, cycles);	//create a new process
		}
	}

	vic[VIC_VADDR] = 1;				//acknowledge the VIC
	c_reschedule();					//reschedule
}

/** @brief Handles a timer interrupt once the state has been saved
  *
  * The node number and port are extracted from the key, the data part is retrieved and a new 
  * processes is created. The pc should never be 0 but this is checked anyway!
  *
  * @param key	an unsigned integer key consisting of the DAMSON node number shifted left 
  * by DAMSONRT_PORT_BITS and the port number/
  */
void c_handle_packet(uint key){
	InterruptVectorItem *iv;
	uint i;
	uint node;
	uint port;
	uint data;
	
  	//get the mc packet key and data
  	data = cc[CC_RXDATA];
  	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] mc pkt: key=%d, payload=%d, ticks=%d\n", key, data, ticks);
		delay_us (API_PRINT_DLY);
	}
	#endif

	//extract node and port, get interrupt, create a new process, schedule
	node = key>>DAMSONRT_PORT_BITS;
	port = key&DAMSON_PORT_MASK;
	iv = get_interrupt(node);
	
	if (iv->code_offset != 0){
		for (i=0; i<iv->count ; i++){
			c_create_process((void*)iv->code_offset, 1, node, port, data, 0);	
		}
		
	}
	vic[VIC_VADDR] = 1;	//ack
	c_reschedule();
}

/** @brief Handles a sdram transfer request once the state has been saved
  *
  * System addresses are offset to the runtime cores unique partition space in SDRAM.If the
  * DMA handler is available then the request is immediately sent otherwise the request is stored in
  * the PCB and the PCB status is set to reflect that the thread is waiting for DMA access. The 
  * cpu_dma_status and dma_handle is used to store the thread waiting for a DMA request to return. 
  * This is required once a DMA request completes. 
  *
  * @param system_address	The relative address (starting from 0x0) to read/write from SDRAM
  * @param local_address	The address to read/write from SDRAM
  * @param words		The number of words to be read/write from SDRAM
  * @param direction		The direction of transfer; either READ (0) or WRITE (1)
  */
void c_handle_sdtransfer(void *system_address, void *local_address, uint words, uint direction)
{
	uint desc;
	desc =   DMA_WIDTH << 24 | DMA_BURST_SIZE << 21 | direction << 19 | (words<<2);
	
	system_address += DAMSONRT_EV_START(virt_cpu);
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		if (direction)
			io_printf(IO_STD, "\t[dib] dma_write: 0x%x, 0x%x, words=%d\n", system_address, local_address, words);
		else
			io_printf(IO_STD, "\t[dib] dma_read: 0x%x, 0x%x, words=%d\n", system_address, local_address, words);
		delay_us (API_PRINT_DLY);
	}
	#endif
	
	//if DMA controller is free core is not waiting for completion then start transfer else 
	if((cpu_dma_status == CPU_DMA_STATUS_FREE)&&(!(dma[DMA_STAT] & 4)))
	{
		dma[DMA_ADRS] = (uint) system_address;
		dma[DMA_ADRT] = (uint) local_address;
		dma[DMA_DESC] = desc;
		CURRENT_PCB->status = PCB_STATUS_DMATRANSFER;
		cpu_dma_status = CPU_DMA_STATUS_DAMSON_TRANSFER;
		dma_handle = CURRENT_PCB->handle;
	}
	else
	{
		CURRENT_PCB->dma.system_address = system_address;
		CURRENT_PCB->dma.local_address = local_address;
		CURRENT_PCB->dma.description = desc;
		CURRENT_PCB->status = PCB_STATUS_DMAWAITING;
	}
	
	//reschedule
	c_reschedule();
}

/** @brief Clears the DMA controller and reschedules
  *
  * A DMA completion may be the result of either a Damson read/write or a log buffer write. Depending on which the stored dma 
  * handle is used to change either the Damson PCB thread state to 'running' or the log buffer state to 'free'. After performing 
  * this operation the status of the dma usage for the cpu is set to 'free'. This indicates that the DMA completion has been 
  * handled and will allow either a new log buffer to be written or a Damson thread waiting for DMA access to start transferring. 
  * Upon completion reschedule is called.
  *
  */
void c_handle_sdcomplete()
{
	dma[DMA_CTRL]  = 0x8;   //clear transfer done interrupt in DMAC 
	
	//handle damson or logging dma transfer differently
	switch (cpu_dma_status){
		case(CPU_DMA_STATUS_DAMSON_TRANSFER):
		{
			#if DAMSONLIB_DEBUG == TRUE
			if (debug_mode){
				io_printf(IO_STD, "\t[dib] damson dma complete: PCB=%d\n", dma_handle);
				delay_us (API_PRINT_DLY);
			}
			#endif
	
			p_list[dma_handle].status = PCB_STATUS_RUNNING;
			break;
		}
		case(CPU_DMA_STATUS_LOGGING_TRANSFER):
		{
			#if DAMSONLIB_DEBUG == TRUE
			if (debug_mode){
				io_printf(IO_STD, "\t[dib] logging dma complete: buffer=%d\n", dma_handle);
				delay_us (API_PRINT_DLY);
			}
			#endif
	
			log_buffers[dma_handle].status = LOG_BUFFER_STATUS_FREE;
			break;
		}
		default:
		{
			#if DAMSONLIB_DEBUG == TRUE
			if (debug_mode){
				io_printf(IO_STD, "\t[dib] Something went wrong with DMA!\n", dma_handle);
				delay_us (API_PRINT_DLY);
			}
			#endif
			break;
		}
	}


	cpu_dma_status = CPU_DMA_STATUS_FREE;	//signal cpu dma transfer is complete
	vic[VIC_VADDR] = 1;			//ack
	
	//check if there are any log buffers to write then re-schedule
	process_log_buffers();
	c_reschedule();
}

/** @brief Sets a PCB to delay by the specified amount
  *
  * The delay in seconds (in 16:16 fixed point notation) must be translated to microseconds and then
  * shifted out of the fixed point format. This is done using a 64 bit (long long int). Any rounding 
  * error within the margin of the tickrate is performed only when delay is less than the tickrate. 
  * This avoids having to perform a modulus operation.
  *
  * @param fp_seconds	A value in seconds in fixed point 16:16 format
  */
void c_handle_delay(int fp_seconds)
{
	//get the delay in microseconds
	long long int fp_us = (long long int)fp_seconds * 1000000;
	fp_us = fp_us/(long long int)(1<<16);
	int us = (int) fp_us;
	
	//rounding by the tickrate (is us is less than tickrate)
	us = round_delay(us, tickrate);	
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] delay: PCB=%d for %d us\n", CURRENT_PCB->handle, (int)us);
		delay_us (API_PRINT_DLY);
	}
	#endif
		
	if (us > 0){	
		CURRENT_PCB->status = PCB_STATUS_DELAYING;
		CURRENT_PCB->delay_us = (int)us;
	}
	
	c_reschedule();
}


/** @brief A function for initialising the VIC
  * 
  * Timer is assigned as an IRQ interrupt, MC packets at FIQ
  */
void init_interrupts(){
	uint int_select;
	uint fiq_select;
	
	// initialize transmitter control to send MC packets
  	cc[CC_TCR] = 0x00000000;
	
	int_select = (1 << DMA_DONE_INT) | (1 << TIMER1_INT);
	fiq_select = (1 << CC_RDY_INT);
		
	// disable interrupt whilst configuring
	vic[VIC_DISABLE] = int_select | fiq_select;
	
	// set packet_interrupt as FIQ
	exception_table[7] = packet_interrupt;

	//configure packet interrupts
	vic_vectors[RX_READY_PRIORITY]  = packet_interrupt;
	vic_controls[RX_READY_PRIORITY] = (0x20 | CC_RDY_INT);

	//configure the dma interrupt
	vic_vectors[DMA_DONE_PRIORITY]  = dma_interrupt;
	vic_controls[DMA_DONE_PRIORITY] = 0x20 | DMA_DONE_INT;

	// configure the timer1 interrupt
	vic_vectors[TIMER1_PRIORITY]  = timer_interrupt;
	vic_controls[TIMER1_PRIORITY] = 0x20 | TIMER1_INT;

	//select and enable interrupts
	vic[VIC_SELECT] = (vic[VIC_SELECT] & ~int_select) | fiq_select;
	vic[VIC_ENABLE] = int_select | fiq_select;	
}

/** @brief Initialises the DMA controller
  * 
  * Initialises the DMA controller
  */
void init_dma_controller()
{
	uint i;
	
	extern_start = (uint*)DAMSONRT_EV_START(virt_cpu);	//start of ev
	extern_end = extern_start + *extern_start;		//ev size (words) is at start of extern vector
	
	cpu_dma_status = CPU_DMA_STATUS_FREE;	//init dma use mode
	
	dma[DMA_CTRL] = 0x3f; // Abort pending and active transfers
	dma[DMA_CTRL] = 0x0d; // clear possible transfer done and restart
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] EV 0x%x to 0x%x (%d words)\n", extern_start, extern_end, *extern_start);
		delay_us (API_PRINT_DLY);
	}
	#endif

	// enable interrupt sources
	dma[DMA_GCTL] = 0x000c00; // enable dma done interrupt

}

/** @brief Initialisation of the routing tables
  *
  * Routing tabels are initialised by the root application processors (for each chip). The 
  * loader will have set an integer with the number of routing entries followed by a set 
  * of key/route tuples in shared SDRAM space immediately after the core map. These are 
  * simply iterated used to set an mc routing table entry with a fixed port mask which 
  * masks the DAMSON MC packet port from the node number.
  */
void init_routing_table()
{
	uint i;
	uint rt_count;
	RoutingEntry *rt;	

	rt_count = *((uint*)DAMSONRT_EV_SHARED_START + chips);					
	rt = (RoutingEntry*)((uint*)DAMSONRT_EV_SHARED_START + chips + 1);

	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf(IO_STD, "\t[dib] routing entries %d at 0x%x\n", rt_count, rt);
		delay_us (API_PRINT_DLY);
	}
	#endif
	for (i=0; i< rt_count; i++)
	{
		set_mc_table_entry(i, rt[i].key, ~DAMSON_PORT_MASK, rt[i].route);
	}
}

/** @brief Initialisation the PCB slots
  *
  * The stack/interrupt stack, size and location are loaded and then checked against the loader 
  * values to ensure there is no mismatch. To initialise the PCB slots the stack is divided between 
  * the number of PCB slots and the reserved SARK/C stack is divided equally amongst the PCBs. Note: 
  * The damson stack grows upwards the C stack downs downwards.
  */

void init_stack()
{
	short int i; 

	//set interrupt stack location in global
	I_STACK_END = (void*)(DAMSONRT_I_STACK_START + DAMSONRT_I_STACK);

	//initialise PCB slots
	for (i=0; i< DAMSONRT_MAX_PROCESSES; i++)
	{
		p_list[i].stack   = ((uint*)DAMSONRT_D_STACK_START)+((i)*WORDS(DAMSONRT_SINGLE_PROCESS_D_STACK)); 	//ascending stack
		p_list[i].c_stack = ((uint*)DAMSONRT_C_STACK_START+ ((i+1)*WORDS(DAMSONRT_SINGLE_PROCESS_C_STACK))); 	//descending stack!
		p_list[i].status = PCB_STATUS_FREE;
		p_list[i].handle = i;
		
		//io_printf (IO_STD, "STACK INIT d_stack=0x%x, c_stack=0x%x\n",  p_list[i].stack, p_list[i].c_stack);
	}
}

void init_overflow_detection()
{
	short int i; 
	uint* c_stack_end;
	
	//set a unique value at the end of the stack
	for (i=0; i< DAMSONRT_MAX_PROCESSES; i++)
	{
		//work out the end of the c stack
		c_stack_end = ((uint*)DAMSONRT_C_STACK_START + ((i)*WORDS(DAMSONRT_SINGLE_PROCESS_C_STACK)));		//end of the descending c stack


		//set unique values to detect stack overflow at the end of the stack
		p_list[i].stack[WORDS(DAMSONRT_SINGLE_PROCESS_D_STACK)-1] = STACK_END_VALUE;	
		*c_stack_end = STACK_END_VALUE;
		
		//io_printf (IO_STD, "STACK ENDS d_stack=0x%x, c_stack=0x%x\n",  &p_list[i].stack[WORDS(DAMSONRT_SINGLE_PROCESS_D_STACK)-1], c_stack_end);

	}
}



/** @brief Initialisation the DAMSON Interrupt hash table
  *
  * Gets the interrupt vector (hash table) and the number of items from the loader 
  * reserved items. The first item is always the timer interrupt (pc == 0 indicates no timer interrupt). The remaining number 
  * of items will always be a power of 2 (to allow a simple modulus function). Code offsets are always offset into the damson 
  * module so this is added to DAMSON_OBJ_BASE, the start of the damson module after linking with damsonlib.
  */
void init_interrupt_vector()
{
	short int i;
	
	//get the interrupt vector location and size
	intvitems = (short int)DAMSONRT_SYSTEM_GLOBAL(5);
	timerint = (InterruptVectorItem*)DAMSONRT_SYSTEM_GLOBAL(40);
	
	//loop through interrupt values and increment code offset
	for (i=0; i<intvitems; i++){
		if (timerint[i].code_offset)
			timerint[i].code_offset += (uint)&DAMSON_OBJ_BASE;
	}
	
	//offset by 1 to ignore first (timer item)	
	intvitems--;
	intv = timerint + 1;
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "\t[dib] DAMSON GV(5) Intv items-1=%d\n", intvitems);
		delay_us(API_PRINT_DLY);
		io_printf (IO_STD, "\t[dib] DAMSON GV(40) Intv=0x%x\n", intv);
		delay_us(API_PRINT_DLY);
	}
	#endif
}


/** @breif init_logging Initialise the logging
  *
  * A LogItem stores the details of a particular log or snapshot (i.e. start and end time and which variables should be logged). 
  * The number of log and snapshot items as well as the start address in memory of the list of log and snapshot LogItems is saved 
  * in the DAMSON reserved global vector. This function retrieves these and stores them in a global. 
  *
  * Any logs have the interval count set so that it can be reduced on a completed clocktick by the tickrate. 
  *
  * The DMA log buffers have the status set to free.
  */
void init_logging()
{
	short int i;
	
	num_logs = (short int)DAMSONRT_SYSTEM_GLOBAL(8);
	num_snapshots = (short int)DAMSONRT_SYSTEM_GLOBAL(9);
	logs = (LogItem*) DAMSONRT_SYSTEM_GLOBAL(43);
	snapshots = (LogItem*) DAMSONRT_SYSTEM_GLOBAL(44);
	
	#if DAMSONLIB_DEBUG == TRUE
	if (debug_mode){
		io_printf (IO_STD, "\t[dib] DAMSON GV(8) logs=%d\n", num_logs);
		delay_us(API_PRINT_DLY);
		io_printf (IO_STD, "\t[dib] DAMSON GV(9) snapshots=%d\n", num_snapshots);
		delay_us(API_PRINT_DLY);
		io_printf (IO_STD, "\t[dib] DAMSON GV(43) log start=0x%x\n", logs);
		delay_us(API_PRINT_DLY);
		io_printf (IO_STD, "\t[dib] DAMSON GV(44) snapshot start=0x%x\n", snapshots);
		delay_us(API_PRINT_DLY);
	}
	#endif
	
	
	//set the interval counts
	for (i=0; i<num_logs; i++)
	{
		logs[i].interval_count = logs[i].interval;
	}
	
	//init the DMA log buffers
	for (i=0; i<MAX_LOG_BUFFERS; i++)
	{
		log_buffers[i].status = LOG_BUFFER_STATUS_FREE;
	}
	
	//get the end of SDRAM for this core (add one as first element of sdram is the gv size)
	log_entry_item = extern_end + 1; 	
	
}


/** @breif log_entry Stores a single log entry to SDRAM
  *
  * Log items are stored at the from the first address in SDRAM after the user external vectors. A log entry has a handle a size 
  * and then a number of log items for each argument in the formatted log string. The handle is used to determine which formatted 
  * string and hence which log file the log entry belongs too. The runtime system does not care about the details of this as it is
  * handled by the loader after execution is complete.
  *
  * This function may be compiled for DMA use or direct access. Direct access will be very slow but might be useful for debugging.
  * Logging via the DMA controller uses a set of log buffers to store a log entry locally. DMA completion will check if there any 
  * log buffers waiting to write via DMA after which it will call a reschedule.
  *
  * @param log	A pointer to a LogItem with the log details
  */
void log_entry(LogItem* log)
{
	uint i, d;
	uint log_words;
	uint *next_log_entry_item;

#ifdef LOGGING_WITH_DMA
	uint desc;
	
	//create the dma description
	log_words = log->log_items + 2;
	desc =   DMA_WIDTH << 24 | DMA_BURST_SIZE << 21 | 1 << 19 | (log_words<<2);

	//get the start of the log entry
	next_log_entry_item = log_entry_item + log_words;
	
	//check that we are not out of log space
	if (next_log_entry_item > (uint*)DAMSONRT_EV_START(virt_cpu+1))
	{
		io_printf(IO_STD, "Warning: Out of log space in SDRAM\n");
		delay_us(API_PRINT_DLY);
		return;
	}

	//find a free log buffer
	for(i=0; i<MAX_LOG_BUFFERS; i++)
	{
		if (log_buffers[i].status == LOG_BUFFER_STATUS_FREE)
		{
						
			//set the log buffer data
			log_buffers[i].status = LOG_BUFFER_STATUS_OCCUPIED;
			log_buffers[i].data[0] = (uint)log->handle;
			log_buffers[i].data[1] = log->log_items;
			for (d=0; d<log->log_items; d++){
				log_buffers[i].data[d+2] = *log->log_globals[d];
			}
			
			
			//if cpu is not waiting to handle a completion and DMA controller is free then start 
			if((cpu_dma_status == CPU_DMA_STATUS_FREE)&&(!(dma[DMA_STAT] & 4)))
			{
				#if DAMSONLIB_DEBUG == TRUE
				if (debug_mode){
					io_printf (IO_STD, "\t[dib] starting dma log handle=%d\n", i);
					delay_us (API_PRINT_DLY);
				}
				#endif
				dma[DMA_ADRS] = (uint) log_entry_item;
				dma[DMA_ADRT] = (uint) log_buffers[i].data;
				dma[DMA_DESC] = desc;
				cpu_dma_status = CPU_DMA_STATUS_LOGGING_TRANSFER;
				dma_handle = i;
			}
			else
			{
				#if DAMSONLIB_DEBUG == TRUE
				if (debug_mode){
					io_printf (IO_STD, "\t[dib] queing dma log handle=%d\n", i);
					delay_us (API_PRINT_DLY);
				}
				#endif
				//update the dma wait item
				log_buffers[i].dma.system_address = log_entry_item;
				log_buffers[i].dma.local_address = log_buffers[i].data;
				log_buffers[i].dma.description = desc;
			}
			
			//set the new log data address
			log_entry_item = next_log_entry_item;
			
			return;
		}
	}
	
	io_printf(IO_STD, "Warning: Log buffers full!\n");
	delay_us(API_PRINT_DLY);

#else	//LOGGING WITHOUT DMA

	uint i;
	uint *next_log_entry_item;
		
	//get the start of the log entry
	next_log_entry_item = log_entry_item + log->log_items + 2;
	
	//check that we are not out of log space
	if (next_log_entry_item > (uint*)DAMSONRT_EV_START(virt_cpu+1))
	{
		io_printf(IO_STD, "Warning: Out of log space in SDRAM\n");
		delay_us(API_PRINT_DLY);
		return;
	}
	
	//log handle, item count and items
	log_entry_item[0] = (uint)log->handle;
	log_entry_item[1] = log->log_items;
	for (d=0; d<log->log_items; d++){
		log_entry_item[d+2] = *log->log_globals[d];
	}
	
	//set the new log data address
	log_entry_item = next_log_entry_item;
#endif
}

/** @breif process_log_buffers Starts a log buffer DMA transfer if possible
  *
  * This function will start writing a waiting log buffer via the DMA if it is possible to do so. It is only possible to 
  * do so if the DMA contoller is free (i.e. not being used by another core), the cpu is not waiting to process the 
  * completion of a previous DMA request and there is an occupied log buffer waiting to write data.
  */
void process_log_buffers()
{
	uint i;

	// dont bother unless DMA is free and not waiting for a DMA completion
	if((cpu_dma_status == CPU_DMA_STATUS_FREE)&&(!(dma[DMA_STAT] & 4))){
		
		//find the next free log buffer
		for(i=0; i<MAX_LOG_BUFFERS; i++)
		{
			if (log_buffers[i].status == LOG_BUFFER_STATUS_OCCUPIED)
			{
				#if DAMSONLIB_DEBUG == TRUE
				if (debug_mode){
					io_printf (IO_STD, "\t[dib] starting queued dma log handle=%d\n", i);
					delay_us (API_PRINT_DLY);
				}
				#endif
				dma[DMA_ADRS] = (uint) log_buffers[i].dma.system_address;
				dma[DMA_ADRT] = (uint) log_buffers[i].dma.local_address;
				dma[DMA_DESC] = log_buffers[i].dma.description;
				cpu_dma_status = CPU_DMA_STATUS_LOGGING_TRANSFER;
				dma_handle = i;
			
				return;
			}
		}
	}
}

/** @breif Hash function
  *
  * Must use the same constants as the loader!
  * Assumes size is a power of 2 to achieve modulus operator on ARM.
  *
  * @param n	A unsigned integer to be hashed.
  * @param size	An unsigned integer specifying the maximum size of the hash table. Must be a power of 2!
  */
uint hash(uint n, uint size)
{
    uint h;
    h = (n * DAMSONRT_HASH_A + DAMSONRT_HASH_C);
    h = h & (size -1);	//x mod y where y is a power 2 = (x & (y − 1))
    return h;
}

/** @ breif Gets the code offset for an interrupt given an interrupting node number
  * 
  * Finds the code offset of a interrupt node number by using a hash lookup.
  *
  * @param node	An unsigned integer representing the node number which caused an interrupt.
  */
InterruptVectorItem* get_interrupt(uint node)
{
	unsigned int n, h;

	n = 0;
	h = hash(node, intvitems);
	while (intv[h].src_node != 0)
	{
		if (node == intv[h].src_node){
			return &intv[h];
		}
		n++;
		if (n >= intvitems){
			io_printf(IO_STD, "Error: Interrupt hash table overflow\n");
			delay_us(API_PRINT_DLY);
		}
		h++;
		if (h >= intvitems){
			h = 0;
		}
	}

	io_printf(IO_STD, "Error: Missing interrupt %d hash table entry\n", node);
	delay_us(API_PRINT_DLY);
	return NULL;

}

/** @breif Simple blocking delay function from spin1 API. 
  *
  * @param us	A delay in microseconds
  */
void delay_us (uint us)
{
	us = (us * sv->cpu_clk) / 4;

	while (us--)
		continue;
}

/** @brief A function for clearing the multicast routing tables
  * 
  */
void clear_mc_tables()
{
	uint i;
	for (i = 0; i < MC_TABLE_SIZE; i++)
	{
		rtr_mckey[i] = 0xffffffff;
		rtr_mcmsk[i] = 0x00000000;
	}
}

/** @brief A function for setting a single mc table entry from spin1 API
  * 
  * @param entry	The index of the routing table entry (will be incremented by SYS_MC_ENTRIES)
  * @param key		The routing key (A DAMSON node number shifted left by DAMSONRT_PORT_BITS)
  * @param mask		A mask to apply to the key (any parts of the key masked must have bit valued of 0). 
  *			This masks outs the DAMSON port in the first DAMSONRT_PORT_BITS bits
  * @param route	This is the routing value (the first 6 bits are offchip routes, remaining bits are index of cores)
  */
void set_mc_table_entry(uint entry, uint key, uint mask, uint route)
{
	if (entry < APP_MC_ENTRIES)
	{
		// need to do virtual to physical core ID conversion
		uint proute = route & (1 << NUM_LINKS) - 1;  // Keep link bits

		for (uint v = 0; v < sv->num_cpus; v++)  // go through virt_cpus
		{
			uint bit = route & (1 << (NUM_LINKS + v));
			if (bit)
			{
				uint pr = sv->v2p_map[v];
				proute |= CORE_ROUTE(pr);
			}
		}

		// top priority entries reserved for the system
		entry += SYS_MC_ENTRIES;

		rtr_mckey[entry] = key;
		rtr_mcmsk[entry] = mask;
		rtr_mcrte[entry] = proute;

		#if DAMSONLIB_DEBUG == TRUE
		if (debug_mode){
			io_printf (IO_STD, "\t[dib] MC %d: k 0x%x m: 0x%x vr 0x%x pr 0x%x\n", entry, key, mask, route, proute);
			delay_us (API_PRINT_DLY);
		}
		#endif
	}else{
		io_printf (IO_STD, "ERROR: Routing table full\n");
		delay_us (API_PRINT_DLY);
	}
	
}


/** @brief A crude modulus function (could be replaced with a binary long division)
  * 
  * @param a	quotient
  * @param b	divisor
  *
  * @return 	The modulus of a%b
  */
uint mod(uint a, uint b)
{
	while (a > b)
		a -= b;
	return a;
}

/** @brief Rounds a delay if it is less than the tickrate
  * Function is used to round a delay when it is less than the tickrate to correct for rounding errors. A delay less than half the 
  * tickrate is rounded down otherwise it is rounded up.
  * 
  * @param delay	the delay value (in microseconds)
  * @param tickrate	the tickrate (in microseconds)
  *
  * @return 	The rounded delay
  */
int round_delay(int delay, uint tickrate)
{
	if (delay < 0)
	{
		return 0;
	}
	
	if (delay < tickrate)
	{
		if(delay < (tickrate>>1))
		{
			delay = 0; //round down (done)
		}
		else
		{
			delay = tickrate; //round up (one tick left)
		}
	}
	return delay;
}



