/*
 * SUMMARY
 *  Implementation of output printing streams for SpiNNaker
 *
 * AUTHOR
 *  Steve Temple - temples@cs.man.ac.uk
 *
 * MODIFIED
 *  Paul Richmond (University of Sheffield) - p.richmond@sheffield.ac.uk
 *
 * COPYRIGHT
 *  Copyright (c) The University of Manchester, 2011. All rights reserved.
 *  SpiNNaker Project
 *  Advanced Processor Technologies Group
 *  School of Computer Science
 */



#ifndef SPINN_IO_H
#define SPINN_IO_H


#define IO_STD 		((char *) 0)		// Stream numbers
#define IO_DBG   	((char *) 1)
#define IO_xxx_2   	((char *) 2)
#define IO_NULL    	((char *) 3)



void io_printf (char *stream, char *f, ...);

#endif
