/*
 * SUMMARY
 *  SARK - Spinnaker Application Runtime Kernel
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
#include "spinn_sdp.h"

#ifdef SARK_API
#include "spin1_api.h"
#include "spin1_api_params.h"
#endif

//------------------------------------------------------------------------------

#ifndef CTIME
#define CTIME 0
#endif

#define VER_NUM  		92
#define VER_STR			"SARK/SpiNNaker"

#define NUM_MSGS		2
#define MAX_MSG			(NUM_MSGS - 1)

#define SHM_IDLE		0
#define SHM_MSG			1
#define SHM_APLX		2
#define SHM_PING		3

#define SHM_ALIVE		0
#define SHM_DEAD		255


//------------------------------------------------------------------------------


extern void cpu_mode_set (uint mode);
extern uint cpu_int_off (void);
extern void snooze (void);
extern void proc_aplx (uint table, uint a2); // really "uint *table"
extern void bx (uint);
extern void copy_msg (sdp_msg_t *to, sdp_msg_t *from);
extern void str_cpy (char *dest, const char *src);

extern void cpu_int (void);
extern void null_int (void);
extern void schedule_sysmode (uchar event_id, uint arg0, uint arg1);


uint virt_cpu;
uint phys_cpu;

static sdp_msg_t msg_bufs[NUM_MSGS];
static sdp_msg_t *msg_root;

static uint msg_count = 0;
static uint msg_max = 0;

static sdp_msg_t *tube_msg;

//------------------------------------------------------------------------------


void sark_error (uint code)
{
  sv->cpu_state[virt_cpu] = code;
  sv->led_period = 8;
  snooze ();
}

//------------------------------------------------------------------------------


static void lock_get (uint lock)
{
  while (sc[SC_TAS0 + lock] & BIT_31)
    continue;
}


static void lock_free (uint lock)
{
  (void) sc[SC_TAC0 + lock];
}


sdp_msg_t* spin1_msg_get ()
{
  uint cpsr = cpu_int_off ();

  sdp_msg_t *msg = msg_root;

  if (msg != NULL)
    {
      msg_root = msg->next;
      msg->next = NULL; // !! Needed??

      msg_count++;
      if (msg_count > msg_max)
	msg_max = msg_count;
    }

  cpu_mode_set (cpsr);

  return msg;
}


void spin1_msg_free (sdp_msg_t *msg)
{
  uint cpsr = cpu_int_off ();

  msg->next = msg_root;
  msg_root = msg;

  msg_count--;

  cpu_mode_set (cpsr);
}


static sdp_msg_t* shm_msg_get ()
{
  uint cpsr = cpu_int_off ();
  lock_get (LOCK_MSG);

  sdp_msg_t *msg = (sdp_msg_t *) sv->msg_root;

  if (msg != NULL)
    {
      sv->msg_root = (msg_t *) msg->next;
      msg->next = NULL;

      sv->msg_count++;
      if (sv->msg_count > sv->msg_max)
	sv->msg_max = sv->msg_count;
    }

  lock_free (LOCK_MSG);
  cpu_mode_set (cpsr);

  return msg;
}


static void shm_msg_free (sdp_msg_t *msg)
{
  uint cpsr = cpu_int_off ();
  lock_get (LOCK_MSG);

  msg->next = (sdp_msg_t *) sv->msg_root;
  sv->msg_root = (msg_t *) msg;

  sv->msg_count--;

  lock_free (LOCK_MSG);
  cpu_mode_set (cpsr);
}


//------------------------------------------------------------------------------


void vic_init (void)
{
  vic[VIC_VADDR] = (uint) vic;			// !! Bodge ?

  vic[VIC_SELECT] = 0x00000000; 		// All use IRQ
  vic[VIC_DISABLE] = 0xffffffff; 		// Disable all

  vic[VIC_DEFADDR] = (uint) null_int;		// Need a handler here (VIC feature!)

  vic[VIC_ADDR0] = (uint) cpu_int;
  vic[VIC_CNTL0] = 0x20 + CPU_INT;

  vic[VIC_ENABLE] = (1 << CPU_INT);

  cpu_mode_set (MODE_SYS);  			// Enables CPU interrupts
}


static void tube_init ()
{
/*
  tube_msg = spin1_msg_get (); // Shouldn't fail here...

  tube_msg->flags = 0x07;
  tube_msg->tag = TAG_HOST;

  tube_msg->dest_port = PORT_ETH; // Take from dbg_addr?
  tube_msg->dest_addr = sv->dbg_addr;

  tube_msg->srce_port = virt_cpu;
  tube_msg->srce_addr = sv->p2p_addr;

  tube_msg->cmd_rc = CMD_TUBE;

  tube_msg->length = 12; // !! const (sdp_hdr + CMD_TUBE)
  */
}


static void msg_init ()
{
  sdp_msg_t *msg = msg_root = msg_bufs;

  for (uint i = 0; i < NUM_MSGS - 1; i++)
    {
      msg->next = msg + 1;
      msg++;
    }

  msg->next = NULL;
}


void sark_setup ()
{
  phys_cpu = dma[DMA_STAT] >> 24;
  virt_cpu = sv->p2v_map[phys_cpu];

  msg_init ();
  tube_init ();
  vic_init ();
}


//------------------------------------------------------------------------------

// Send msg to MP

uint spin1_send_sdp_msg (sdp_msg_t *msg, uint timeout)
{
  mbox_t* mbox = mbox_mp + virt_cpu;

  if (mbox->state == SHM_DEAD)
    return 0;

  sdp_msg_t *shm_msg = shm_msg_get ();

  if (shm_msg == NULL)
    return 0;

  copy_msg (shm_msg, msg);

//interupts off and lock moved to here
  uint cpsr = cpu_int_off ();
  lock_get (LOCK_MBOX);
  
  mbox->msg = (msg_t *) shm_msg;
  mbox->cmd = SHM_MSG;

//interrupt off and lock moved from here

  sv->mbox_flags |= 1 << virt_cpu;

  lock_free (LOCK_MBOX);
  cpu_mode_set (cpsr);

  volatile uint *ms = &sv->clock_ms;
  uint start = *ms;

  while (mbox->cmd != SHM_IDLE)
    if (*ms - start > timeout)
      break;

  if (mbox->cmd != SHM_IDLE)
    {
      shm_msg_free (shm_msg);
      mbox->state = SHM_DEAD;
      return 0;
    }

  return 1;
}


void svc_putc (uchar c, char *stream) 	// stream ignored for now
{
  uchar *buf = (uchar *) tube_msg + 8;	// Point at start of msg

  buf[tube_msg->length++] = c;		// Insert char at end

  if (c == 0 || c == '\n' || tube_msg->length == BUF_SIZE + 12) // !! const
    {
      spin1_send_sdp_msg (tube_msg, 1000); 	// !! const
      tube_msg->length = 12; 		// !! const
    }
}



//------------------------------------------------------------------------------


static uint cmd_ver (sdp_msg_t *msg)
{
  msg->arg1 = (sv->p2p_addr << 16) + (phys_cpu << 8) + virt_cpu;
  msg->arg2 = (VER_NUM << 16) + BUF_SIZE;
  msg->arg3 = CTIME;

  str_cpy ((char *) msg->data, VER_STR);

  return 12 + sizeof (VER_STR);
}


static uint cmd_read (sdp_msg_t *msg) // arg1=addr, arg2=len, arg3=type
{
  uint len = msg->arg2;
  uint type = msg->arg3;

  if (len > BUF_SIZE || type > TYPE_WORD)
    {
      msg->cmd_rc = RC_ARG;
      return 0;
    }

  uint addr = msg->arg1;
  uchar *buffer = (uchar *) &msg->arg1;

  if (type == TYPE_BYTE)
    {
      uchar *mem = (uchar *) addr;
      uchar *buf = (uchar *) buffer;

      for (uint i = 0; i < len; i++)
	buf[i] = mem[i];
    }
  else if (type == TYPE_HALF)
    {
      ushort *mem = (ushort *) addr;
      ushort *buf = (ushort *) buffer;

      for (uint i = 0; i < len / 2; i++)
	buf[i] = mem[i];

    }
  else
    {
      uint *mem = (uint *) addr;
      uint *buf = (uint *) buffer;

      for (uint i = 0; i < len / 4; i++)
	buf[i] = mem[i];
    }

  return len;
}


static uint cmd_write (sdp_msg_t *msg) // arg1=addr, arg2=len, arg3=type
{
  uint len = msg->arg2;
  uint type = msg->arg3;

  if (len > BUF_SIZE || type > TYPE_WORD)
    {
      msg->cmd_rc = RC_ARG;
      return 0;
    }

  uint addr = msg->arg1;
  uchar *buffer = msg->data;

  if (type == TYPE_BYTE)
    {
      uchar *mem = (uchar*) addr;
      uchar *buf = (uchar*) buffer;

      for (uint i = 0; i < len; i++)
	mem[i] = buf[i];
    }
  else if (type == TYPE_HALF)
    {
      ushort* mem = (ushort*) addr;
      ushort* buf = (ushort*) buffer;

      for (uint i = 0; i < len / 2; i++)
	mem[i] = buf[i];
    }
  else
    {
      uint *mem = (uint *) addr;
      uint *buf = (uint *) buffer;

      for (uint i = 0; i < len / 4; i++)
	mem[i] = buf[i];
    }

  return 0;
}


static uint cmd_run (sdp_msg_t *msg)
{
  bx (msg->arg1);

  return 0;
}


static uint debug (sdp_msg_t *msg)
{
  uint len = msg->length;

  if (len < 24) // !! const
    {
      msg->cmd_rc = RC_LEN;
      return 12;
    }

  uint t = msg->cmd_rc;
  msg->cmd_rc = RC_OK;

  switch (t)
    {
    case CMD_VER:
      return 12 + cmd_ver (msg);

    case CMD_RUN:
      return 12 + cmd_run (msg);

    case CMD_READ: 
      return 12 + cmd_read (msg);

    case CMD_WRITE: 
      return 12 + cmd_write (msg);

    case CMD_APLX:
      proc_aplx (msg->arg1, 0); // Never returns
      return 12;

    default:
      msg->cmd_rc = RC_CMD;
      return 12;
    }
}


//------------------------------------------------------------------------------


static void swap_sdp_hdr (sdp_msg_t *msg)
{
  uint dest_port = msg->dest_port;
  uint dest_addr = msg->dest_addr;

  msg->dest_port = msg->srce_port;
  msg->srce_port = dest_port;

  msg->dest_addr = msg->srce_addr;
  msg->srce_addr = dest_addr;
}


void cpu_int_c ()
{
  sc[SC_CLR_IRQ] = SC_CODE + (1 << phys_cpu);

  mbox_t *mbox = mbox_ap + virt_cpu;
  uint cmd = mbox->cmd;
  sdp_msg_t *shm_msg = (sdp_msg_t *) mbox->msg;

  if (cmd == SHM_MSG)
    {
      mbox->cmd = SHM_IDLE;

      sdp_msg_t *msg = spin1_msg_get ();

      if (msg != NULL)
	{
	  copy_msg (msg, shm_msg);
	  shm_msg_free (shm_msg);

	  uint dp = msg->dest_port;

	  if ((dp & 0xe0) == 0) // Port 0 is for us !! const
	    {
	      msg->length = debug (msg);
	      swap_sdp_hdr (msg);

	      spin1_send_sdp_msg (msg, 1000);
	      spin1_msg_free (msg);
	    }
	  else	  // else send msg to application...
	    {
#ifdef SARK_API
	      if (callback[SDP_PACKET_RX].cback != NULL)
		{
		  uint cpsr = cpu_int_off ();
		  schedule_sysmode (SDP_PACKET_RX, (uint) msg, dp >> 5); // !! const
		  cpu_mode_set (cpsr);
		}
	      else
		spin1_msg_free (msg);
#else
	      spin1_msg_free (msg);
#endif
	    }
	}
      else
	{
	  shm_msg_free (shm_msg);
	}
    }
  else if (cmd == SHM_APLX)
    {
      mbox->cmd = SHM_IDLE;
      proc_aplx ((uint) shm_msg, 0);
    }
  else if (cmd == SHM_PING)
    {
      uint p = (uint) shm_msg;
      p += 1;
      mbox->msg = (msg_t *) p;
      mbox->cmd = SHM_IDLE;
    }
}

//------------------------------------------------------------------------------
