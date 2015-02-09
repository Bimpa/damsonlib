#
# Makefile for damsonlib using GNU tools
#

THUMB=no
CTIME = $(shell perl -e "print time")

SRC_DIR = ./src
LIB_DIR = ./lib
INC_DIR = ./include

CC := arm-none-linux-gnueabi-gcc -c -O1 -nostdlib -mthumb-interwork \
      -march=armv5te -std=gnu99 -I $(INC_DIR)
CT := $(CC)
AS := arm-none-linux-gnueabi-as -I $(INC_DIR) -mthumb-interwork -march=armv5te --defsym GNU=1
LD := arm-none-linux-gnueabi-ld
OC := arm-none-linux-gnueabi-objcopy

ifeq ($(THUMB),yes)
  CT += -mthumb -DTHUMB
  AS += --defsym THUMB=1
endif


API_OBJS = $(LIB_DIR)/sark_init.o  $(LIB_DIR)/sark.o  $(LIB_DIR)/cdamsonrt.o  $(LIB_DIR)/damsonrt.o  $(LIB_DIR)/spinn_io.o  $(LIB_DIR)/spinn_sync.o

lib: $(API_OBJS)
	$(LD) -T damsonlib.lnk -nostdlib -i -o $(LIB_DIR)/damsonlib.o $(API_OBJS)

$(INC_DIR)/spinnaker.gas: $(INC_DIR)/spinnaker.h
	h2asm $(INC_DIR)/spinnaker.h | arm2gas > $(INC_DIR)/spinnaker.gas

$(LIB_DIR)/sark_init.o: $(INC_DIR)/spinnaker.gas $(SRC_DIR)/sark_init.gas
	$(AS) $(SRC_DIR)/sark_init.gas -o $(LIB_DIR)/sark_init.o
	
$(LIB_DIR)/sark.o:  $(SRC_DIR)/sark.c $(INC_DIR)/spinnaker.h
	$(CT) -DCTIME=$(CTIME) $(SRC_DIR)/sark.c -o $(LIB_DIR)/sark.o

$(LIB_DIR)/spinn_io.o:  $(SRC_DIR)/spinn_io.c $(INC_DIR)/spinn_io.h $(INC_DIR)/spinnaker.h
	$(CT)  $(SRC_DIR)/spinn_io.c -o $(LIB_DIR)/spinn_io.o
	
$(LIB_DIR)/spinn_sync.o: $(SRC_DIR)/spinn_sync.c $(INC_DIR)/spinn_io.h $(INC_DIR)/spinnaker.h
	$(CT)  $(SRC_DIR)/spinn_sync.c -o $(LIB_DIR)/spinn_sync.o

$(LIB_DIR)/cdamsonrt.o: $(SRC_DIR)/cdamsonrt.c $(INC_DIR)/spinnaker.h $(INC_DIR)/spinn_io.h $(INC_DIR)/damsonrt.h
	$(CT) $(SRC_DIR)/cdamsonrt.c -o $(LIB_DIR)/cdamsonrt.o

$(LIB_DIR)/damsonrt.o: $(SRC_DIR)/damsonrt.gas $(INC_DIR)/spinnaker.h $(SRC_DIR)/sark_init.gas
	$(AS) $(SRC_DIR)/damsonrt.gas -o $(LIB_DIR)/damsonrt.o



clean:
	rm -f $(LIB_DIR)/damsonlib.o $(API_OBJS) $(INC_DIR)/spinnaker.gas
