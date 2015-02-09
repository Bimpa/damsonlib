/*
 * SUMMARY
 *  Syncronisation module factored from the SpiNNaker API for use with the damson runtime
 *
 * AUTHORS
 *  Steve Temple (University of Manchester) - temples@cs.man.ac.uk
 *  Modified by Paul Richmond (University of Sheffield) - p.richmond@sheffield.ac.uk
 *
 *
 * COPYRIGHT
 *  Copyright (C)    The University of Manchester - 2010, 2011
 *  SpiNNaker Project
 */
 
#include "spinnaker.h"
#include "spinn_io.h"
#include "damsonrt.h"



/* debug output */
#define API_DEBUG FALSE //DAMSONLIB_DEBUG

/* previously in header */
#define SYNC_PKT				(0x1ffff << 11)

/* synchronization barrier key and mask codes */
#define GO_KEY                	(uint) (SYNC_PKT | 0)
#define RDYGO_KEY             	(uint) (SYNC_PKT | 1)
#define RDY1_KEY              	(uint) (SYNC_PKT | 2)
#define RDY2_KEY              	(uint) (SYNC_PKT | 3)
#define BARRIER_MASK          	(uint) (0xffffffff)
#define BARRIER_GO_WAIT       	30000
#define BARRIER_RDY2_WAIT     	30000
#define BARRIER_RDYGO_WAIT    	30000
#define BARRIER_LOCK_WAIT     	30000
#define BARRIER_RESEND_WAIT   	100

/* link orientation codes */
#define EAST                  	0
#define NORTH_EAST            	1
#define NORTH                 	2
#define WEST                  	3
#define SOUTH_WEST            	4
#define SOUTH                 	5

/* sync macros */
#define CHIP_ADDR(x, y)       	((x << 8) | y)
#define P2P_ROUTE(addr)      	(1 << p2p_get(addr))
#define CORE_ROUTE(core)     	(1 << (core + NUM_LINKS))

#define ROUTER_INIT_WAIT      	30000
#define NON_ROOT 		FALSE

/* lock usage */
#define CLEAR_LCK             0
#define RTR_INIT_LCK          0xad
#define RDYGO_LCK             0xbe



/* damson c externals */
extern void delay_us(uint us);
extern uchar rootAp;


/* local (non external) functions */
void set_core_map(uint chips, uint * core_map);
void barrier_setup(uint chips, uint * core_map);
uint p2p_get (uint entry);
void send_sync_packet(uint key);

/* Sync count and signals (from SpiNNaker API) */ 
volatile uint 		barrier_rdy1_cnt;
volatile uint 		barrier_rdy2_cnt;
volatile uint 		barrier_go = FALSE;
volatile uint 		barrier_rdygo = FALSE;
static uint   		my_chip = 0;            // chip address in core_map coordinates
static volatile uint 	my_ncores = 0;   	// this chip's appl. cores in the sim.
uint			chips;		 	// number of chips in layout
static volatile uint 	nchips = 1;      	// chips in the simulation
static volatile uint 	ncores = 1;      	// application cores in the simulation
uchar  			rootChip;               // flag for root chip
static ushort 		rootAddr;               // address of root chip

/* SARK externals */
extern uint virt_cpu;
extern uint phys_cpu;

/* router MC tables */
static volatile uint * const rtr_mcrte = (uint *) RTR_MCRAM_BASE;
static volatile uint * const rtr_mckey = (uint *) RTR_MCKEY_BASE;
static volatile uint * const rtr_mcmsk = (uint *) RTR_MCMASK_BASE;

/* router P2P tables */
static volatile uint * const rtr_p2p   = (uint *) RTR_P2P_BASE;

/* VIC vector tables */
typedef void (*isr_t) (void);
#define EXCEPTION_BASE 0x00000020
static volatile isr_t * const vic_vectors  = (isr_t *) (VIC_BASE + 0x100);
static volatile uint * const vic_controls = (uint *) (VIC_BASE + 0x200);
static isr_t * const exception_table = (isr_t *) EXCEPTION_BASE;





/*************************** external procedures below here **************************/

void sync_init(uint c, uint *cm)
{
	unsigned int* core_map;
	uint i;
	uint h;
	
	chips = c;
	core_map = cm;
	
	if (rootAp)
	{
		
		// setup the RDY1 entry
		// local ready packets come to me
		rtr_mckey[2] = RDY1_KEY;
		rtr_mcmsk[2] = BARRIER_MASK;
		rtr_mcrte[2] = CORE_ROUTE(phys_cpu);

		// router initialized -- let other cores go!
		sv->lock = RTR_INIT_LCK;

		// initialize ready counts (I'm already ready!)
		barrier_rdy1_cnt = 1;
		barrier_rdy2_cnt = 1;

		#if (API_DEBUG == TRUE)
			io_printf (IO_STD, "\t[sync] RDY1: k 0x%8z m 0x%8z r 0x%8z\n", RDY1_KEY, BARRIER_MASK, CORE_ROUTE(phys_cpu));
			delay_us (API_PRINT_DLY);
		#endif
		
	}else
	{
		uint start;
		volatile uint *ms_cnt = &sv->clock_ms;

		/* wait for router init to finish */
		// timeout if taking too long!
		start = *ms_cnt;          // initial value
		while (((*ms_cnt - start) < ROUTER_INIT_WAIT) && (sv->lock < RTR_INIT_LCK))
		{
			continue;
		}


		if (sv->lock < RTR_INIT_LCK)
		{
			io_printf (IO_STD, "Warning: waiting for router init failed.\n");
			delay_us (API_PRINT_DLY);
		}

	}	

	//set the core map
	set_core_map(chips, core_map);
}




void sync_handle_packet(uint key)
{
  	
	#if API_DEBUG == TRUE
		io_printf(IO_STD, "\t[sync] mc sync packet: key=%d\n", key);
		delay_us (API_PRINT_DLY);
	#endif
	
	switch (key)
	{
		case RDYGO_KEY:
			barrier_rdygo = TRUE;
			break;
		case GO_KEY:
			#if API_DEBUG == TRUE
				io_printf(IO_STD, "\t[sync] SETTING BARRIER GO: %d\n", barrier_go);
				delay_us (API_PRINT_DLY);
			#endif
			barrier_go = TRUE;
			#if API_DEBUG == TRUE
				io_printf(IO_STD, "\t[sync] SET BARRIER GO: %d\n", barrier_go);
				delay_us (API_PRINT_DLY);
			#endif
			break;
		case RDY1_KEY:
			barrier_rdy1_cnt++;
			break;
		case RDY2_KEY:
			barrier_rdy2_cnt++;
			break;
		default:
			#if API_DEBUG == TRUE
				io_printf(IO_STD, "\t[sync] sync packet key unrecognised: key=%d\n", key);
				delay_us (API_PRINT_DLY);
			#endif
			break;
	}
	
	
	//ack and return
	vic[VIC_VADDR] = 1;
}



void sync_barrier_wait(void) {
	uint start;
	uint resend;
	volatile uint *ms_cnt = &sv->clock_ms;
	
	// easy out
	if (ncores == 1)
		return;

	if (rootAp) {
		// let other cores go!
		sv->lock = RDYGO_LCK;

		if (rootChip) {
			// root node, root application core
			// send rdygo packet
			send_sync_packet(RDYGO_KEY);

			// wait until all ready packets arrive and give the go signal
			// timeout if taking too long!
			start = *ms_cnt; // initial value
			resend = start; // may need to resend rdygo
			while (((*ms_cnt - start) < (BARRIER_RDY2_WAIT)) && ((barrier_rdy1_cnt < my_ncores) || (barrier_rdy2_cnt< nchips))) {
				if ((*ms_cnt - resend) > BARRIER_RESEND_WAIT) {
					// send a new rdygo packet -- just in case the first was missed!
					send_sync_packet(RDYGO_KEY);
					resend = *ms_cnt;
				}
			}
			if ((barrier_rdy1_cnt != my_ncores) || (barrier_rdy2_cnt != nchips)) {
				io_printf(IO_STD, "\t[api_warn] warning: failed to synchronise (%d/%d) (%d/%d).\n", 
				                  barrier_rdy1_cnt, my_ncores, barrier_rdy2_cnt, nchips);
				delay_us(API_PRINT_DLY);
			}


			// send go packet
			send_sync_packet(GO_KEY);
		} else {
			// non-root node, root application core
			// wait until the rdygo packet and all local ready packets arrive
			// timeout if taking too long!
			start = *ms_cnt; // initial value
			while (((*ms_cnt - start) < (BARRIER_RDYGO_WAIT)) && ((barrier_rdy1_cnt < my_ncores) || (!barrier_rdygo))) {
				continue;
			}
			if (barrier_rdy1_cnt != my_ncores) {
				io_printf(IO_STD, "\t[api_warn] warning: failed to synchronise (%d/%d).\n",
						  barrier_rdy1_cnt, my_ncores);
				delay_us(API_PRINT_DLY);
			} else if (!barrier_rdygo) {
				io_printf(IO_STD, "\t[api_warn] warning: synchronisation failed (rdygo).\n");
				delay_us(API_PRINT_DLY);
			}

			// send rdy2 packet
			send_sync_packet(RDY2_KEY);
			#if API_DEBUG == TRUE
				io_printf(IO_STD, "\t[sync] Sending rdy2...\n");
			#endif
		}
	} else {
		// all others
		// wait for lock -- from local root application core
		// timeout if taking too long!
		start = *ms_cnt; // initial value
		while (((*ms_cnt - start) < BARRIER_LOCK_WAIT)
				&& (sv->lock < RDYGO_LCK)) {
			continue;
		}
		if (sv->lock < RDYGO_LCK) {

			io_printf(IO_STD, "\t[api_warn] warning: synchronisation failed (lock).\n");
			delay_us(API_PRINT_DLY);
		}

		// send local ready packet
		#if API_DEBUG == TRUE
			if (NON_ROOT) {
				io_printf(IO_STD, "\t[sync] Sending ready...\n");
			}
		#endif
		send_sync_packet(RDY1_KEY);

	}

	// wait until go packet arrives
	// timeout if taking too long!
	start = *ms_cnt; // initial value
	while (((*ms_cnt - start) < BARRIER_GO_WAIT) && (barrier_go == FALSE)) {
		continue;
	}

	if (barrier_go == FALSE) {
		io_printf(IO_STD, "\t[api_warn] warning: synchronisation timeout (go).%d\n", barrier_go);
		delay_us(API_PRINT_DLY);
	}


	//reset for next sync
	if (rootAp) {
		barrier_rdy1_cnt = 1;
		barrier_rdy2_cnt = 1;
	}
	barrier_rdygo = FALSE;
	barrier_go = FALSE;
}

/*************************** local procedures below here **************************/

void set_core_map(uint chips, uint * core_map)
{
	uint i, j;
	uint nc = 0;
	uint cores;

	// count the number of cores
	for (i = 0; i < chips; i++)
	{
		cores = core_map[i];
		/* exclude monitor -- core 0 */
		for (j = 1; j < sv->num_cpus; j++)
		{
			if (cores & (1 << j)) nc++;
		}
	}
	ncores = nc;
	
	
	#if API_DEBUG == TRUE
		if ((NON_ROOT) || rootAp)
		{
			io_printf (IO_STD, "\t[sync] ROOT: chips: %d core_map: %x num_cores: %d\n", chips, core_map, ncores);
			delay_us (API_PRINT_DLY);
		}
	#endif


	// check if root chip (MOVED FROM barrier setup)
	rootChip = (sv->p2p_addr == CHIP_ADDR(0, 0));
	rootAddr = CHIP_ADDR(0, 0);

	// if needed, setup mc routing table to implement barrier synchronisation
	// only the root application core does the setup
	if ((ncores > 1) && (rootAp))
	{
		barrier_setup(chips, core_map);
	}
}

void barrier_setup(uint chips, uint* core_map)
{
	uint i;
	uint my_cores = 0;

	// TODO: needs extending -- works for square core maps only!
	uint bsize = 0;
	uint bside = 1;
	for (i = 0; i < 32; i++){
		if ((chips & (0x80000000 >> i)) != 0){
			bsize = (31 - i) >> 1;
			bside = 1 << bsize;
			break;
		}
	}

	#if API_DEBUG == TRUE
		io_printf (IO_STD, "\t[sync] bside: %d, bsize: %d\n", bside, bsize);
		delay_us (API_PRINT_DLY);
	#endif

	// TODO: needs extending -- works for square core maps only!
	uint my_x = sv->p2p_addr >> 8;
	uint my_y = sv->p2p_addr & 0xff;
	my_chip = (my_x << bsize) | my_y;



	// setup routing entries for synchronization barrier
	uint loc_route = 0;
	uint off_route = 0;
	uint rdygo_route;
	uint rdy2_route;

	// setup the local (on-chip) routes
	my_cores = core_map[my_chip] & 0xfffffffe;  // exclude monitor
	// need to do virtual to physical core ID conversion
	for (uint v = 1; v < sv->num_cpus; v++){  // go through virt_cpus
		if (my_cores & (1 << v))
		{
			loc_route |= CORE_ROUTE(sv->v2p_map[v]);
			my_ncores++;
		}
	}
	#if API_DEBUG == TRUE
		io_printf (IO_STD, "\t[sync]  my chip: %d, my core map: 0x%x\n", my_chip, my_cores);
		delay_us (API_PRINT_DLY);
	#endif

	// TODO: needs fixing -- fault-tolerant tree
	// setup off-chip routes -- check for borders!
	// north
	if ((my_x == 0) && (my_y < (bside - 1)) && ((core_map[my_chip + 1] & 0xfffffffe) != 0)){
		off_route |= (1 << NORTH); // add link to north chip
	}

	// east
	if ((my_x < (bside - 1)) && (my_y == 0) && ((core_map[my_chip + bside] & 0xfffffffe) != 0)){
		off_route |= (1 << EAST); // add link to east chip
	}

	// north-east
	if ((my_y < (bside - 1)) && (my_x < (bside - 1)) && ((core_map[my_chip + bside + 1] & 0xfffffffe) != 0)) {
		off_route |= (1 << NORTH_EAST); // add link to north-east chip
	}

	// TODO: needs fixing -- non-fault-tolerant tree
	// TODO: doesn't use wrap around
	// spanning tree from root chip N, E & NE
	if (rootChip){
		// compute number of active chips
		for (i = 0; i < chips; i++){
			if ((i != my_chip) && ((core_map[i] & 0xfffffffe) != 0)){
				nchips++;
			}
		}

		// setup the RDYGO entry -- only off-chip routes!
		rdygo_route = off_route;
		// setup the RDY2 route -- packets come to me
		rdy2_route = CORE_ROUTE(phys_cpu);
	}
	else
	{
		// setup the RDYGO entry -- packets to me and forward
		rdygo_route = CORE_ROUTE(phys_cpu) | off_route;
		// setup the RDY2 route -- packets go to root chip (use p2p routing table to find the way)
		rdy2_route = P2P_ROUTE(rootAddr);
	}

	// setup the RDYGO entry
	rtr_mckey[1] = RDYGO_KEY;
	rtr_mcmsk[1] = BARRIER_MASK;
	rtr_mcrte[1] = rdygo_route;

	// setup the RDY2 entry
	rtr_mckey[3] = RDY2_KEY;
	rtr_mcmsk[3] = BARRIER_MASK;
	rtr_mcrte[3] = rdy2_route;

	// setup the GO entry
	rtr_mckey[0] = GO_KEY;
	rtr_mcmsk[0] = BARRIER_MASK;
	rtr_mcrte[0] = loc_route | off_route;

	#if API_DEBUG == TRUE
		io_printf (IO_STD, "\t[sync] RDYGO: k 0x%8z m 0x%8z r 0x%8z\n", RDYGO_KEY, BARRIER_MASK, rdygo_route);
		delay_us (API_PRINT_DLY);

		io_printf (IO_STD, "\t[sync] RDY2: k 0x%8z m 0x%8z r 0x%8z\n", RDY2_KEY, BARRIER_MASK, rdy2_route);
		delay_us (API_PRINT_DLY);

		io_printf (IO_STD, "\t[sync] GO: k 0x%8z m 0x%8z r 0x%8z\n", GO_KEY, BARRIER_MASK, loc_route | off_route);
		delay_us (API_PRINT_DLY);
	#endif
}




uint p2p_get (uint entry)
{
	uint word = entry >> P2P_LOG_EPW;

	if (word >= P2P_TABLE_SIZE)
	return 6;

	uint offset = P2P_BPE * (entry & P2P_EMASK);
	uint data = rtr_p2p[word];

	return (data >> offset) & P2P_BMASK;
}




void send_sync_packet (uint key)
{
	//lock until free
	while ((cc[CC_TCR] & BIT_31) == 0)
	{
		continue;
	}
	#if API_DEBUG == TRUE
		io_printf(IO_STD, "\t[sync] sendpkt: key=0x%x stream=0x%a\n", key);
		delay_us(API_PRINT_DLY);
	#endif
	cc[CC_TCR] = TCR_MC;
	cc[CC_TXKEY] = key;
}



