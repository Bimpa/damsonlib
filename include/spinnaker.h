/*
 * SUMMARY
 *  SpiNNaker peripherals address main header file
 *
 * AUTHOR
 *  Steve Temple - temples@cs.man.ac.uk
 *
 *
 * COPYRIGHT
 *  Copyright (c) The University of Manchester, 2011. All rights reserved.
 *  SpiNNaker Project
 *  Advanced Processor Technologies Group
 *  School of Computer Science
 */


#ifndef SPINNAKER_H
#define SPINNAKER_H

typedef unsigned int uint;
typedef unsigned short ushort;
typedef unsigned char uchar;

#define ntohs(t) ((((t) & 0x00ff) << 8) | (((t) & 0xff00) >> 8))
#define htons(t) ((((t) & 0x00ff) << 8) | (((t) & 0xff00) >> 8))

#define INT_AT(t) ((t) / 4)
#define BYTE_AT(t) (t)

//------------------------------------------------------------------------------

// Numbers of CPUS, links

#ifdef TEST_CHIP
#define NUM_CPUS		2
#else
#define NUM_CPUS		18
#endif

#define MAX_CPU 		(NUM_CPUS - 1)

#define NUM_LINKS		6
#define MAX_LINK		(NUM_LINKS - 1)

//------------------------------------------------------------------------------

// Memory definitions

#define ITCM_BASE		0x00000000
#define ITCM_SIZE		0x00008000
#define ITCM_TOP		(ITCM_BASE + ITCM_SIZE)

#define ITCM_TOP_64		(ITCM_TOP - 64)
#define ITCM_TOP_512		(ITCM_TOP - 512)

#define DTCM_BASE		0x00400000
#define DTCM_SIZE		0x00010000
#define DTCM_TOP		(DTCM_BASE + DTCM_SIZE)

#define SDRAM_BASE		0x70000000
#define SDRAM_SIZE		(128 * 1024 * 1024)
#define SDRAM_TOP		(SDRAM_BASE + SDRAM_SIZE)

#define SYSRAM_BASE 		0xf5000000

#ifdef TEST_CHIP
#define SYSRAM_SIZE 		0x00004000
#else
#define SYSRAM_SIZE 		0x00008000
#endif

#define SYSRAM_TOP 		(SYSRAM_BASE + SYSRAM_SIZE)

#define ROM_BASE		0xf6000000
#define ROM_SIZE		0x00008000
#define ROM_TOP			(ROM_BASE + ROM_SIZE)

//------------------------------------------------------------------------------

// Comms controller definitions

#define CC_BASE			0x10000000	// Unbuffered

#ifdef TEST_CHIP
#define CC_TCR			INT_AT(0x00)
#define CC_RSR			INT_AT(0x04)
#define CC_TXDATA		INT_AT(0x08)
#define CC_TXKEY		INT_AT(0x0c)
#define CC_RXDATA		INT_AT(0x10)
#define CC_RXKEY		INT_AT(0x14)
#define CC_TEST 		INT_AT(0x1c)
#else
#define CC_TCR			INT_AT(0x00)
#define CC_TXDATA		INT_AT(0x04)
#define CC_TXKEY		INT_AT(0x08)
#define CC_RSR			INT_AT(0x0c)
#define CC_RXDATA		INT_AT(0x10)
#define CC_RXKEY		INT_AT(0x14)
#define CC_SAR 			INT_AT(0x18)
#define CC_TEST 		INT_AT(0x1c)
#endif

//------------------------------------------------------------------------------

// Timer definitions

#define TIMER_BASE		0x11000000	// Unbuffered

#define T1_LOAD			INT_AT(0x00)
#define T1_COUNT		INT_AT(0x04)
#define T1_CONTROL		INT_AT(0x08)
#define T1_INT_CLR		INT_AT(0x0c)
#define T1_RAW_INT		INT_AT(0x10)
#define T1_MASK_INT		INT_AT(0x14)
#define T1_BG_LOAD		INT_AT(0x18)

#define T2_LOAD			INT_AT(0x20)
#define T2_COUNT		INT_AT(0x24)
#define T2_CONTROL		INT_AT(0x28)
#define T2_INT_CLR		INT_AT(0x2c)
#define T2_RAW_INT		INT_AT(0x30)
#define T2_MASK_INT		INT_AT(0x34)
#define T2_BG_LOAD		INT_AT(0x38)

//------------------------------------------------------------------------------

// VIC definitions

#define VIC_BASE		0x1f000000	// Unbuffered

#define VIC_IRQST		INT_AT(0x00)
#define VIC_FIQST		INT_AT(0x04)
#define VIC_RAW			INT_AT(0x08)
#define VIC_SELECT		INT_AT(0x0c)
#define VIC_ENABLE		INT_AT(0x10)
#define VIC_DISABLE		INT_AT(0x14)
#define VIC_SOFT_SET		INT_AT(0x18)
#define VIC_SOFT_CLR		INT_AT(0x1c)
#define VIC_PROTECT 		INT_AT(0x20)
#define VIC_VADDR		INT_AT(0x30)
#define VIC_DEFADDR		INT_AT(0x34)

#define VIC_ADDR0		INT_AT(0x100)
#define VIC_ADDR1		INT_AT(0x104)
#define VIC_ADDR2		INT_AT(0x108)
#define VIC_ADDR3		INT_AT(0x10c)
#define VIC_ADDR4		INT_AT(0x110)
#define VIC_ADDR5		INT_AT(0x114)
#define VIC_ADDR6		INT_AT(0x118)
#define VIC_ADDR7		INT_AT(0x11c)
#define VIC_ADDR8		INT_AT(0x120)
#define VIC_ADDR9		INT_AT(0x124)
#define VIC_ADDR10		INT_AT(0x128)
#define VIC_ADDR11		INT_AT(0x12c)
#define VIC_ADDR12		INT_AT(0x130)
#define VIC_ADDR13		INT_AT(0x134)
#define VIC_ADDR14		INT_AT(0x138)
#define VIC_ADDR15		INT_AT(0x13c)

#define VIC_CNTL0		INT_AT(0x200)
#define VIC_CNTL1		INT_AT(0x204)
#define VIC_CNTL2		INT_AT(0x208)
#define VIC_CNTL3		INT_AT(0x20c)
#define VIC_CNTL4		INT_AT(0x210)
#define VIC_CNTL5		INT_AT(0x214)
#define VIC_CNTL6		INT_AT(0x218)
#define VIC_CNTL7		INT_AT(0x21c)
#define VIC_CNTL8		INT_AT(0x220)
#define VIC_CNTL9		INT_AT(0x224)
#define VIC_CNTL10		INT_AT(0x228)
#define VIC_CNTL11		INT_AT(0x22c)
#define VIC_CNTL12		INT_AT(0x230)
#define VIC_CNTL13		INT_AT(0x234)
#define VIC_CNTL14		INT_AT(0x238)
#define VIC_CNTL15		INT_AT(0x23c)

#define WDOG_INT		0
#define SOFTWARE_INT		1
#define COMM_RX_INT		2
#define COMM_TX_INT		3
#define TIMER1_INT		4
#define TIMER2_INT		5
#define CC_RDY_INT		6
#define CC_RPE_INT		7
#define CC_RFE_INT		8
#define CC_TFL_INT		9
#define CC_TOV_INT		10
#define CC_TMT_INT		11
#define DMA_DONE_INT		12
#define DMA_ERR_INT		13
#define DMA_TO_INT		14
#define RTR_DIAG_INT		15
#define RTR_DUMP_INT		16
#define RTR_ERR_INT		17
#define CPU_INT			18
#define ETH_TX_INT		19
#define ETH_RX_INT		20
#define ETH_PHY_INT		21
#define SLOW_CLK_INT		22

#ifdef TEST_CHIP
#else
#define CC_TNF_INT		23
#define CC_MC_INT		24
#define CC_P2P_INT		25
#define CC_NN_INT		26
#define CC_FR_INT		27
#define EXT0_INT		28
#define EXT1_INT		29
#define EXT2_INT		30
#define EXT3_INT		31
#endif

//------------------------------------------------------------------------------

// DMA controller definitions

#define DMA_BASE		0x30000000	// Unbuffered

#ifdef TEST_CHIP
#define DMA_CRC  		INT_AT(0x00)
#define DMA_CRC2		INT_AT(0x100)
#else
#define DMA_CRCT		INT_AT(0x180)
#endif

#define DMA_ADRS 		INT_AT(0x04)
#define DMA_ADRT 		INT_AT(0x08)
#define DMA_DESC 		INT_AT(0x0c)
#define DMA_CTRL 		INT_AT(0x10)
#define DMA_STAT 		INT_AT(0x14)
#define DMA_GCTL 		INT_AT(0x18)
#define DMA_CRCC 		INT_AT(0x1c)
#define DMA_CRCR 		INT_AT(0x20)
#define DMA_TMTV 		INT_AT(0x24)
#define DMA_SCTL 		INT_AT(0x28)

#define DMA_STAT0 		INT_AT(0x40)
#define DMA_STAT1 		INT_AT(0x44)
#define DMA_STAT2 		INT_AT(0x48)
#define DMA_STAT3 		INT_AT(0x4c)
#define DMA_STAT4 		INT_AT(0x50)
#define DMA_STAT5 		INT_AT(0x54)
#define DMA_STAT6 		INT_AT(0x58)
#define DMA_STAT7 		INT_AT(0x5c)

#define DMA_AD2S		INT_AT(0x104)
#define DMA_AD2T		INT_AT(0x108)
#define DMA_DES2		INT_AT(0x10c)

//------------------------------------------------------------------------------

// PL340 definitions

#define PL340_BASE		0xf0000000

#define MC_STAT			INT_AT(0x00)
#define MC_CMD 			INT_AT(0x04)
#define MC_DIRC 		INT_AT(0x08)
#define MC_MCFG 		INT_AT(0x0c)

#define MC_REFP 		INT_AT(0x10)
#define MC_CASL 		INT_AT(0x14)
#define MC_DQSS 		INT_AT(0x18)
#define MC_MRD			INT_AT(0x1c)
#define MC_RAS			INT_AT(0x20)
#define MC_RC			INT_AT(0x24)
#define MC_RCD			INT_AT(0x28)
#define MC_RFC			INT_AT(0x2c)
#define MC_RP			INT_AT(0x30)
#define MC_RRD			INT_AT(0x34)
#define MC_WR			INT_AT(0x38)
#define MC_WTR			INT_AT(0x3c)
#define MC_XP			INT_AT(0x40)
#define MC_XSR			INT_AT(0x44)
#define MC_ESR			INT_AT(0x48)

#define MC_MCFG2		INT_AT(0x4c)
#define MC_MCFG3		INT_AT(0x50)

#define MC_QOS0			INT_AT(0x100)
#define MC_QOS1			INT_AT(0x104)
#define MC_QOS2			INT_AT(0x108)
#define MC_QOS3			INT_AT(0x10c)
#define MC_QOS4			INT_AT(0x110)
#define MC_QOS5			INT_AT(0x114)
#define MC_QOS6			INT_AT(0x118)
#define MC_QOS7			INT_AT(0x11c)
#define MC_QOS8			INT_AT(0x120)
#define MC_QOS9			INT_AT(0x124)
#define MC_QOS10		INT_AT(0x128)
#define MC_QOS11		INT_AT(0x12c)
#define MC_QOS12		INT_AT(0x130)
#define MC_QOS13		INT_AT(0x134)
#define MC_QOS14		INT_AT(0x138)
#define MC_QOS15		INT_AT(0x13c)

#define MC_CCFG0		INT_AT(0x200)
#define MC_CCFG1		INT_AT(0x204)
#define MC_CCFG2		INT_AT(0x208)
#define MC_CCFG3		INT_AT(0x20c)

#define DLL_STATUS		INT_AT(0x300)
#define DLL_CONFIG0		INT_AT(0x304)
#define DLL_CONFIG1		INT_AT(0x308)

//------------------------------------------------------------------------------

// Router definitions

#define RTR_BASE 		0xf1000000

#define RTR_CONTROL		INT_AT(0x00)
#define RTR_STATUS		INT_AT(0x04)

#define RTR_EHDR		INT_AT(0x08)
#define RTR_EKEY		INT_AT(0x0c)
#define RTR_EDAT		INT_AT(0x10)
#define RTR_ESTAT		INT_AT(0x14)
#define RTR_DHDR		INT_AT(0x18)
#define RTR_DKEY		INT_AT(0x1c)
#define RTR_DDAT		INT_AT(0x20)
#define RTR_DLINK		INT_AT(0x24)
#define RTR_DSTAT		INT_AT(0x28)
#define RTR_DGEN		INT_AT(0x2c)

#define RTR_DGF0		INT_AT(0x200)
#define RTR_DGF1		INT_AT(0x204)
#define RTR_DGF2		INT_AT(0x208)
#define RTR_DGF3		INT_AT(0x20c)
#define RTR_DGF4		INT_AT(0x210)
#define RTR_DGF5		INT_AT(0x214)
#define RTR_DGF6		INT_AT(0x218)
#define RTR_DGF7		INT_AT(0x21c)

#define RTR_DGC0		INT_AT(0x300)
#define RTR_DGC1		INT_AT(0x304)
#define RTR_DGC2		INT_AT(0x308)
#define RTR_DGC3		INT_AT(0x30c)
#define RTR_DGC4		INT_AT(0x310)
#define RTR_DGC5		INT_AT(0x314)
#define RTR_DGC6		INT_AT(0x318)
#define RTR_DGC7		INT_AT(0x31c)

#ifdef TEST_CHIP
#else
#define RTR_DGF8		INT_AT(0x220)
#define RTR_DGF9		INT_AT(0x224)
#define RTR_DGF10		INT_AT(0x228)
#define RTR_DGF11		INT_AT(0x22c)
#define RTR_DGF12		INT_AT(0x230)
#define RTR_DGF13		INT_AT(0x234)
#define RTR_DGF14		INT_AT(0x238)
#define RTR_DGF15		INT_AT(0x23c)

#define RTR_DGC8		INT_AT(0x320)
#define RTR_DGC9		INT_AT(0x324)
#define RTR_DGC10		INT_AT(0x328)
#define RTR_DGC11		INT_AT(0x32c)
#define RTR_DGC12		INT_AT(0x330)
#define RTR_DGC13		INT_AT(0x334)
#define RTR_DGC14		INT_AT(0x338)
#define RTR_DGC15		INT_AT(0x33c)
#endif

#define RTR_TST1		INT_AT(0xf00)
#define RTR_TST2		INT_AT(0xf04)

#ifdef TEST_CHIP
#define P2P_TABLE_SIZE 		256
#define MC_TABLE_SIZE     	256
#else
#define P2P_TABLE_SIZE 		8192
#define MC_TABLE_SIZE     	1024
#endif

#define MC_RAM_WIDTH    	(NUM_CPUS + NUM_LINKS)
#define MC_RAM_MASK		((1 << MC_RAM_WIDTH) - 1)

#define P2P_EPW			8		// Entries per word
#define P2P_LOG_EPW		3		// Log of entries per word
#define P2P_EMASK		(P2P_EPW-1)	// Entries per word - 1
#define P2P_BPE			3		// Bits per entry
#define P2P_BMASK		7		// Mask for entry bits

#define P2P_INIT 		0x00db6db6	// All thrown away!

#define RTR_P2P_BASE 		(RTR_BASE + 0x00010000)
#define RTR_P2P_TOP 		(RTR_P2P_BASE + P2P_TABLE_SIZE * 4)

#define RTR_MCRAM_BASE 		(RTR_BASE + 0x00004000)
#define RTR_MCRAM_TOP 		(RTR_MCRAM_BASE + MC_TABLE_SIZE * 4)
#define RTR_MCKEY_BASE 		(RTR_BASE + 0x00008000)
#define RTR_MCMASK_BASE		(RTR_BASE + 0x0000c000)

#define MC_CAM_WIDTH    	32

//------------------------------------------------------------------------------

// System controller definitions

#define SYSCTL_BASE		0xf2000000

#define SC_CODE			0x5ec00000

#ifdef TEST_CHIP
#define CHIP_ID			0x59100902
#else
#define CHIP_ID			0x59111012
#endif

#define SC_CHIP_ID		INT_AT(0x00)
#ifdef TLM
#define SC_TUBE			INT_AT(0x00)
#endif
#define SC_CPU_DIS		INT_AT(0x04)
#define SC_SET_IRQ		INT_AT(0x08)
#define SC_CLR_IRQ		INT_AT(0x0c)
#define SC_SET_OK		INT_AT(0x10)
#define SC_CPU_OK		INT_AT(0x10)
#define SC_CLR_OK		INT_AT(0x14)

#define SC_SOFT_RST_L		INT_AT(0x18)
#define SC_HARD_RST_L     	INT_AT(0x1c)
#define SC_SUBS_RST_L     	INT_AT(0x20)

#define SC_SOFT_RST_P		INT_AT(0x24)
#define SC_HARD_RST_P     	INT_AT(0x28)
#define SC_SUBS_RST_P     	INT_AT(0x2c)

#define SC_RST_CODE		INT_AT(0x30)
#define SC_MON_ID		INT_AT(0x34)

#define SC_MISC_CTRL    	INT_AT(0x38)

#ifdef TEST_CHIP
#define SC_MISC_STAT    	INT_AT(0x3c)
#else
#define GPIO_RES		INT_AT(0x3c)
#endif

#define GPIO_PORT		INT_AT(0x40)
#define GPIO_DIR		INT_AT(0x44)
#define GPIO_SET		INT_AT(0x48)
#define GPIO_CLR		INT_AT(0x4c)
#define GPIO_READ		INT_AT(0x48)

#define SC_PLL1			INT_AT(0x50)
#define SC_PLL2			INT_AT(0x54)

#ifdef TEST_CHIP
#define SC_TORIC1		INT_AT(0x58)
#define SC_TORIC2		INT_AT(0x5c)
#else
#define SC_FLAG			INT_AT(0x58)
#define SC_SETFLAG		INT_AT(0x58)
#define SC_CLRFLAG		INT_AT(0x5c)
#endif

#define SC_CLKMUX		INT_AT(0x60)
#define SC_SLEEP		INT_AT(0x64)

#ifdef TEST_CHIP
#else
#define SC_TS0			INT_AT(0x68)
#define SC_TS1			INT_AT(0x6c)
#define SC_TS2			INT_AT(0x70)
#endif

#define SC_ARB0			INT_AT(0x080)
#define SC_TAS0			INT_AT(0x100)
#define SC_TAC0			INT_AT(0x180)

#ifdef TEST_CHIP
#define SC_MISC_TEST		INT_AT(0x200)
#define SC_LINK_DIS		INT_AT(0x204)
#else
#define SC_LINK_DIS		INT_AT(0x200)
#endif

#define RST_POR			0
#define RST_WDT			1
#define RST_USER		2
#define RST_SW			3
#define RST_WDI			4

//------------------------------------------------------------------------------

// Watchdog timer definitions

#define WDOG_BASE		0xf3000000

#define WD_LOAD   		INT_AT(0x00)
#define WD_COUNT  		INT_AT(0x04)
#define WD_CTRL   		INT_AT(0x08)
#define WD_INTCLR 		INT_AT(0x0c)
#define WD_RAWINT 		INT_AT(0x10)
#define WD_MSKINT 		INT_AT(0x14)
#define WD_LOCK   		INT_AT(0xc00)

#define WD_CODE   		0x1acce551

#define WD_INT_B  		1
#define WD_RST_B  		2

//------------------------------------------------------------------------------

// Ethernet definitions

#define ETH_BASE		0xf4000000

#define ETH_TX_BASE		(ETH_BASE + 0x0000)
#define ETH_TX_SIZE		0x0600
#define ETH_TX_TOP		(ETH_TX_BASE + ETH_TX_SIZE)

#define ETH_RX_BASE		(ETH_BASE + 0x4000)
#define ETH_RX_SIZE		0x0c00
#define ETH_RX_TOP		(ETH_RX_BASE + ETH_RX_SIZE)

#define ETH_RX_DESC_RAM		(ETH_BASE + 0x8000)
#define ETH_REGS		(ETH_BASE + 0xc000)

#define	ETH_CONTROL		INT_AT(0x00)
#define ETH_STATUS    		INT_AT(0x04)
#define	ETH_TX_LEN    		INT_AT(0x08)
#define	ETH_TX_CMD     		INT_AT(0x0c)
#define	ETH_RX_CMD     		INT_AT(0x10)
#define	ETH_MAC_LO  		INT_AT(0x14)
#define	ETH_MAC_HI   		INT_AT(0x18)
#define ETH_PHY_CTRL    	INT_AT(0x1c)
#define ETH_INT_CLR		INT_AT(0x20)
#define	ETH_RX_BUF_RP		INT_AT(0x24)
#define	ETH_RX_DESC_RP		INT_AT(0x2c)

#define ETH_TX_CLR		0x01
#define ETH_RX_CLR		0x10

// Bits in ETH_PHY_CTRL

#define PHY_CTRL_NRST 		1
#define PHY_CTRL_DIN  		2
#define PHY_CTRL_DOUT 		4
#define PHY_CTRL_OE   		8
#define PHY_CTRL_CLK  		16

//------------------------------------------------------------------------------

// Spinnaker packet definitions

#define PKT_MC			0x00000000
#define PKT_P2P			0x00400000
#define PKT_NN			0x00800000
#define PKT_NND			0x00a00000
#define PKT_FR			0x00c00000

#define PKT_MASK		0x00c00000

#define P2P_TYPE_BITS		0x00030000

#ifdef TEST_CHIP
#define TCR_MC			(PKT_MC  + 0x07000000)
#define TCR_P2P			(PKT_P2P + 0x07000000)
#define TCR_NN  		(PKT_NN  + 0x07000000)
#define TCR_NND 		(PKT_NND + 0x07000000)
#define TCR_FR  		(PKT_FR  + 0x07000000)
#else
#define TCR_MC			PKT_MC
#define TCR_P2P			PKT_P2P
#define TCR_NN  		PKT_NN
#define TCR_NND 		PKT_NND
#define TCR_FR 			PKT_FR
#endif

#define TCR_PAYLOAD 		0x00020000
#define RSR_PAYLOAD 		0x00020000

#define TCR_MC_P 		(TCR_MC  + TCR_PAYLOAD)
#define TCR_P2P_P 		(TCR_P2P + TCR_PAYLOAD)
#define TCR_NN_P 		(TCR_NN  + TCR_PAYLOAD)
#define TCR_NND_P 		(TCR_NND + TCR_PAYLOAD)
#define TCR_FR_P 		(TCR_FR  + TCR_PAYLOAD)

//------------------------------------------------------------------------------

// Handy constants that point to hardware

static volatile uint * const sc = (uint *) SYSCTL_BASE;
static volatile uint * const cc = (uint *) CC_BASE;
static volatile uint * const tc = (uint *) TIMER_BASE;
static volatile uint * const vic = (uint *) VIC_BASE;
static volatile uint * const dma = (uint *) DMA_BASE;
static volatile uint * const rtr = (uint *) RTR_BASE;
static volatile uint * const er = (uint *) ETH_REGS;
static volatile uint * const mc = (uint *) PL340_BASE;
static volatile uint * const wd = (uint *) WDOG_BASE;

//------------------------------------------------------------------------------

// PHY registers

#define	PHY_CONTROL		0
#define	PHY_STATUS		1
#define	PHY_ID1			2
#define PHY_ID2			3
#define PHY_AUTO_ADV		4
#define PHY_AUTO_LPA		5
#define PHY_AUTO_EXP		6
#define PHY_SI_REV		16
#define PHY_MODE_CSR		17
#define PHY_SP_MODE		18
#define PHY_ERR_COUNT		26
#define PHY_SP_CSIR		27
#define PHY_INT_SRC		29
#define	PHY_INT_MASK		30
#define	PHY_SP_CSR		31

//------------------------------------------------------------------------------

// ARM CPSR bits

#define MODE_USER		0x10
#define MODE_FIQ		0x11
#define MODE_IRQ		0x12
#define MODE_SVC		0x13
#define MODE_ABT		0x17
#define MODE_UND		0x1b
#define MODE_SYS		0x1f

#define THUMB_BIT		0x20

#define IMASK_IRQ		0x80
#define IMASK_FIQ		0x40
#define IMASK_ALL		0xc0

//------------------------------------------------------------------------------

// Default stack pointers

#define IRQ_STACK		DTCM_TOP
#define IRQ_SIZE		256

#define FIQ_STACK               (IRQ_STACK - IRQ_SIZE)
#define FIQ_SIZE                256

#define SVC_STACK               (FIQ_STACK - FIQ_SIZE)

//------------------------------------------------------------------------------

// Misc definitions

#define MAX_CPUS 		20			// Do not exceed!

#define BUF_SIZE 		256			// Size of SDP buffer

#define NULL			0			// Null pointer value

#define MONITOR_CPU 		0			// Virtual CPU number

#define ROM_IMAGE1 		0xffff4000		// Check this

#define BIT_31			0x80000000		// Bit number defs
#define BIT_30			0x40000000
#define BIT_29			0x20000000
#define BIT_0			0x00000001

#define DEAD_WORD		0xdeaddead	       	// Pad word value

#define SW_UNKNOWN		0			// Software versions
#define SW_SCAMP		1

//------------------------------------------------------------------------------

// APLX definitions

#define APLX_END		0xffffffff

#define APLX_ACOPY		1
#define APLX_RCOPY		2
#define APLX_FILL		3
#define APLX_EXEC		4

#define APLX_SIZE		16

//------------------------------------------------------------------------------

#define P2P_TYPE_SDP		(0 << 16)

#define P2P_CTRL		(1 << 24)

#define P2P_OPEN_REQ   		(9 << 24)
#define P2P_OPEN_ACK   		(1 << 24)
#define P2P_DATA_ACK   		(3 << 24)
#define P2P_CLOSE_REQ  		(5 << 24)
#define P2P_CLOSE_ACK  		(7 << 24)

#define P2P_DEF_SQL  		4			// Seq len = 2^4

//------------------------------------------------------------------------------

#define CMD_VER    		0
#define CMD_RUN   		1
#define CMD_READ   		2
#define CMD_WRITE  		3
#define CMD_APLX  		4

#define CMD_AP_MAX		15

#define CMD_LINK_PROBE 		16
#define CMD_LINK_READ  		17
#define CMD_LINK_WRITE 		18
#define CMD_xxx_19 		19

#define CMD_NNP  		20
#define CMD_P2PC 		21
#define CMD_PING  		22
#define CMD_FFD 		23

#define CMD_AS			24
#define CMD_LED 		25
#define CMD_IPTAG 		26
#define CMD_SROM  		27

#define CMD_TUBE  		64

#define RC_OK 			0x80	// Command completed OK
#define RC_LEN 			0x81	// Bad packet length
#define RC_SUM 			0x82	// Bad checksum
#define RC_CMD 			0x83	// Bad/invalid command
#define RC_ARG     		0x84	// Invalid arguments
#define RC_PORT	 		0x85	// Bad port number
#define RC_TIMEOUT 		0x86	// Timeout
#define RC_ROUTE 		0x87	// No P2P route
#define RC_CPU	 		0x88	// Bad CPU number
#define RC_DEAD	 		0x89	// SHM dest dead
#define RC_BUF			0x8a	// No free SHM buffers
#define RC_P2P_NOREPLY		0x8b	// No reply to open
#define RC_P2P_REJECT		0x8c	// Open rejected
#define RC_P2P_BUSY		0x8d	// Dest busy
#define RC_P2P_TIMEOUT		0x8e	// Dest died?
#define RC_PKT_TX		0x8f	// Pkt Tx failed

#define TYPE_BYTE 		0
#define TYPE_HALF 		1
#define TYPE_WORD 		2

//------------------------------------------------------------------------------

// IPTAG definitions

#define IPTAG_NEW		0
#define IPTAG_SET		1
#define IPTAG_GET		2
#define IPTAG_CLR		3
#define IPTAG_AUTO		4

#define IPTAG_MAX		IPTAG_AUTO

#define IPTAG_VALID		0x8000	// Entry is valid
#define IPTAG_TRANS		0x4000	// Entry is transient
#define IPTAG_ARP		0x2000	// Awaiting ARP resolution

#define TAG_NONE  		255	// Invalid tag/transient request
#define TAG_HOST 		0	// Reserved for host

//------------------------------------------------------------------------------

// Bits in SDP Flags byte

#define SDPF_REPLY		0x80	// Spare
#define SDPF_xxx_40		0x40	// Spare
#define SDPF_SUM		0x20	// Checksum before routing
#define SDPF_DP2P		0x10	// Disable P2P check in routing
#define SDPF_DLINK		0x08	// Disable Link check in routing
#define SDPF_LMASK		0x07	// Link bits mask

//------------------------------------------------------------------------------

// Allcations of SysCtl Test & Set registers (locks)

#define LOCK_MSG		0
#define LOCK_MBOX		1
#define LOCK_ETHER		2
#define LOCK_GPIO		3
#define LOCK_API_ROOT		4

//------------------------------------------------------------------------------

// NN opcodes
// Codes < 8 have propagation limited by the ID field in the packet.
// Codes >= 8 have various other ways of handling propagation.
// Codes with bit 2 clear (0-3, 8-11) have explicit FwdRty in the packet.
// Codes with bit 2 set (4-7, 12-15) use stored FwdRty parameters

#define NN_CMD_SIG0		0	// Misc (GTPC, Set FwdRty, LED, etc)
#define NN_CMD_RTRC 		1	// Router Control Reg
#define NN_CMD_LTPC		2	// Local Time Phase Ctrl (ID=0, Fwd=0)
#define NN_CMD_SP_3		3

#define NN_CMD_SIG1     	4	// Misc (MEM, etc)
#define NN_CMD_P2PC 		5	// P2P Address setup
#define NN_CMD_FFS   		6	// Flood fill start
#define NN_CMD_SP_7		7

#define NN_CMD_PING 		8	// Hop count limited
#define NN_CMD_P2PB		9	// Hop count limited
#define NN_CMD_SDP		10	// ** Handled specially
#define NN_CMD_SP_11 		11	// Spare

#define NN_CMD_FBS 		12	// Filtered in FF code
#define NN_CMD_FBD 		13
#define NN_CMD_FBE 		14
#define NN_CMD_FFE 		15

#define NN_SDP_KEY		((0x50 + NN_CMD_SDP) << 24)

//------------------------------------------------------------------------------

// Clock & PLL definitions

#define CLK_XTAL_MHZ		10		// Crystal frequency (MHz)

#define PLL_300			0x0007011e	// Assuming 10MHz in
#define PLL_400			0x00070128	//

#define PLL_260			0x0007011a	//
#define PLL_330			0x00070121	//

#define PLL_LOCK_TIME		80		// Microseconds

#ifdef TEST_CHIP
#define PLL_CLK_SEL		0x9b000165	// CPU/2, SYS/3, RTR/2, MEM/1
#else
#define PLL_CLK_SEL		0x809488a5	// CPU/2, SYS/3, RTR/3, MEM/1
#endif

//------------------------------------------------------------------------------

// Bits in GPIO[31:0]

// Serial ROM

#define SERIAL_SO		0x04		// In
#define SERIAL_SI		0x08		// Out
#define SERIAL_CLK		0x10		// Out
#define SERIAL_NCS		0x20		// Out

#define SERIAL_OE		(SERIAL_NCS + SERIAL_CLK + SERIAL_SI)


// Hardware (PCB) versions

#define GPIO_HW_VER		0x3f		// HW version on 5:2

#define HW_VER_S2		0		// Spin2 Test Card
#define HW_VER_S3		8		// Spin3 (Bunny) Card


// LED definitions

#define LED_0			0x01		// Green LED on bit 0

#define LED_ON(n)		(3 << (2 * n))
#define LED_OFF(n)		(2 << (2 * n))
#define LED_INV(n)		(1 << (2 * n))


// On-chip SDRAM

#define SDRAM_TQ		BIT_31
#define SDRAM_DPD		BIT_30
#define SDRAM_HERE		BIT_29

// Handshaking IO with ARM board

#define SER_OUT_0		0x01
#define SER_OUT_1		0x02

#define SER_OUT			(SER_OUT_0 + SER_OUT_1)

#define SER_IN_0		0x40
#define SER_IN_1		0x80

#define SER_IN			(SER_IN_0 + SER_IN_1)

//------------------------------------------------------------------------------

// Failure codes 

// Non-zero causes CPU to sleep
// Bit 6 set causes CPU_OK bit to be cleared
// Bit 5 set causes LED3 to be turned on

// HW errors have bit 7 set & bit 6 set
// SW errors have bit 7 set & bit 6 clr

#define FAIL_RESET		0xc0		// Catch-all - set at reset
#define FAIL_ROMX		0xc1		// Exception in ROM code
#define FAIL_ITCM0		0xc2		// ITCM top 512 failure
#define FAIL_ITCM1		0xc3		// ITCM main test failure
#define FAIL_DTCM		0xc4		// DTCM test failure

#define FAIL_TIMER		0xc5		// Timer reg test failed
#define FAIL_VIC		0xc6		// VIC reg test failed
#define FAIL_CC			0xc7		// Comms ctlr reg test failed
#define FAIL_DMA		0xc8		// DMAC reg test failed

#define FAIL_MP			0xc9		// Previous monitor proc failure
#define FAIL_LATE		0xca		// App CPU failed to set CPU_OK
#define FAIL_MANUF		0xcb		// App CPU in manuf test
#define FAIL_SLEEP		0xcc		// Ordered to sleep in startup

#define FAIL_TLM		0xcf		// Special for TLM

#define FAIL_VEC		0xa0		// Unhandled exception

//------------------------------------------------------------------------------

// Assorted typedefs

typedef struct			// Returned (div, mod) from divide
{
  uint div;
  uint mod;
} divmod_t;


typedef struct			// SDP header
{
  uchar flags;
  uchar tag;
  uchar dest_port;
  uchar srce_port;
  ushort dest_addr;
  ushort srce_addr;
  //  uint data[];
} sdp_hdr_t;


typedef struct			// Command header
{
  ushort cmd_rc;
  ushort flags;
  uint arg1;
  uint arg2;
  uint arg3;
  //  uint buf[];
} cmd_hdr_t;


typedef struct msg		// Message
{
  struct msg *next;
  ushort length;
  ushort checksum;
  uchar buf[284]; // sdp_hdr, cmd_hdr, BUF_SIZE, PAD4
} msg_t;


typedef struct mbox		// Mailbox (8 bytes)
{
  volatile uchar cmd;
  uchar state;
  volatile uchar flag;
  uchar _PAD;
  void *msg;
} mbox_t;


typedef struct srom_data	//  Contents of SV SROM area (32 bytes)
{
  ushort flags;
  uchar  mac_addr[6];
  uchar  ip_addr[4];
  uchar  gw_addr[4];
  uchar  net_mask[4];
  ushort udp_port;
  ushort pad0;
  uint   pad1;
  uint   pad2;
} srom_data_t;


typedef struct 			// IPTAG entry (16 bytes)
{
  uchar ip[4];
  uchar mac[6];
  ushort port;
  ushort timeout;
  ushort flags;
} iptag_t;

//------------------------------------------------------------------------------

// SV (System) RAM definitions - these are at f5003xxx in TEST_CHIP

#define SV_SSIZE		32			// Initialised from SROM
#define SV_USIZE		96			// Not initialised
#define SV_ISIZE		128			// Initialised to 0
#define SV_VSIZE		32			// Reset vectors
#define SV_PSIZE		64			// Spare
#define SV_RSIZE		64			// Random
#define SV_MSIZE		(8 * MAX_CPUS)		// MP MBOXes (20 @ 8 bytes)
#define SV_ASIZE		(8 * MAX_CPUS)		// AP MBOXes (20 @ 8 bytes)

#define SV_SBASE   		(SYSRAM_TOP - SV_SSIZE)	// f5007fe0
#define SV_UBASE		(SV_SBASE - SV_USIZE)	// f5007f80
#define SV_IBASE    		(SV_UBASE - SV_ISIZE)	// f5007f00
#define SV_BASE			SV_IBASE		// f5007f00

#define SV_VBASE		(SV_IBASE - SV_VSIZE)	// f5007ee0
#define SV_PBASE		(SV_IBASE - SV_PSIZE)	// f5007ea0 BUG but do not fix !!
#define SV_RBASE		(SV_PBASE - SV_RSIZE)	// f5007e60
#define SV_ABASE		(SV_RBASE - SV_ASIZE)	// f5007dc0
#define SV_MBASE		(SV_ABASE - SV_MSIZE)	// f5007d20

// Offsets from SV_BASE

#define SROM_FLAG_BASE		(SV_SBASE + 0)		// f5007fe0
#define STATUS_MAP_BASE		(SV_UBASE + 0) 		// f5007f60
#define RST_BLOCK_BASE		(SV_VBASE + 0)         	// f5007ee0

// Bits in srom_data->flags

#define SRF_PRESENT		0x8000			// SROM present
#define SRF_HW_VER		0x00f0			// Hardware version
#define SRF_PHY_INIT		0x0008			// Init PHY on startup
#define SRF_PHY_RST		0x0004			// Reset PHY on startup
#define SRF_HI_LED		0x0002			// LEDs active high
#define SRF_ETH  		0x0001			// Ethernet present
#define SRF_NONE  		0x0000			// None of the above

// Pointers to ap_mbox[], mp_mbox[] in SYSRAM

static mbox_t * const mbox_ap = (mbox_t *) SV_ABASE;	
static mbox_t * const mbox_mp = (mbox_t *) SV_MBASE;

// struct which sits at top of SysRAM

typedef struct
{
  ushort p2p_addr;	// 00
  ushort p2p_dims;	// 02
  ushort dbg_addr;	// 04
  uchar  p2p_up;	// 06
  uchar  last_id;	// 07
  ushort ltpc_period;	// 08
  uchar hw_ver;		// 0a
  uchar spare_c1;	// 0b

  uchar link_up;	// 0c
  uchar p2p_sql;	// 0d
  uchar chip_state;	// 0e
  uchar tp_scale;	// 0f

  uint clock_ms;	// 10
  ushort time_ms;	// 14
  ushort spare_s2;	// 16
  uint time_sec;	// 18
  uint tp_timer;	// 1c

  ushort cpu_clk;	// 20
  ushort mem_clk;	// 22
  ushort rtr_clk;	// 24
  ushort sys_clk;	// 26

  uchar forward;	// 28
  uchar retry;		// 29
  uchar peek_time;	// 2a
  uchar led_period;	// 2b
  ushort p2pb_period;	// 2c
  ushort probe_period;	// 2e

  void *msg_root;	// 30
  uint sdram_size;	// 34
  uint mem_ptr;		// 38
  uint random;		// 3c
  uchar cpu_state[20];	// 40

  uchar open_req_retry;	// 54 Tx open_req
  uchar open_ack_retry;	// 55 Rx open_ack
  uchar data_ack_retry;	// 56 Rx data_ack
  uchar close_req_retry;// 57 Rx close_req

  ushort open_ack_time;	// 58 Tx awaiting open_ack
  ushort data_ack_time;	// 5a Tx awaiting data ack
  ushort data_time;	// 5c Rx awaiting data
  ushort close_ack_time;// 5e Rx awaiting close_ack

  uint mbox_flags;	// 60
  uint man_test;	// 64
  uint msg_count;	// 68
  uint msg_max;		// 6c

  iptag_t tube_iptag;	// 70

  uchar status_map[20];	// 80
  uchar p2v_map[20];	// 94
  uchar v2p_map[20];	// a8
  volatile uchar lock;	// bc
  uchar num_cpus;	// bd
  ushort spare_s3;	// be
  uint utmp0;		// c0
  uint utmp1;		// c4
  uint utmp2;		// c8
  uint utmp3;		// cc
  uint u_spare[4];	// d0
  srom_data_t srom;	// e0
} sv_t;

static sv_t * const sv = (sv_t *) SV_BASE;		// Points to SV struct in SYSRAM

#endif //..//
