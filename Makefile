include makedefs
OUTFILE=firmware

# Include all sub directories  use regex to exlude hidden dirs
INCLUDEDIRS=$(shell find * \( ! -regex '.*/\..*' \) -type d)
INCLUDE=-I.
INCLUDE+=$(addprefix -I, $(INCLUDEDIRS))

CFLAGS+= $(INCLUDE)

CFLAGS+=-D'BAUDRATE=115200'

VPATH=lpc1xxx:libs:core
VPATH+=$(addprefix :, $(INCLUDEDIRS))

OBJFILES = uart.o cpu_init.o core_cm0.o i2c.o bma180.o itg3200.o hmc5843.o rc_pwm.o

OBJS = $(addprefix ${OBJFOLDER}/, $(OBJFILES))

INIT_OBJS= 


#  ##########################################################################
#  #LPC21ISP PROGRAMMING
#  ##########################################################################
#  LPC21ISP = /usr/local/bin/lpc21isp
#  LPC21ISP_PORT = /dev/tty.usbserial*
#  LPC21ISP_BAUD = 115200 #38400
#  LPC21ISP_XTAL = 12000
#  LPC21ISP_FLASHFILE = $(OBJFOLDER)/$(OUTFILE).hex
#  LPC21ISP_CONTROL = -control -RstDone
#  LPC21ISP_OPTIONS = -verify -term 




# Make all
##########################################################################
all: ${OBJFOLDER}           \
     stuff 

# Clean the stuff 
##########################################################################
clean:
	@rm -rf ${OBJFOLDER} ${wildcard *.bin} $(OUTFILE).elf


# Make the folder used to hold the objs
##########################################################################
${OBJFOLDER}:
	@mkdir ${OBJFOLDER}

# Make the efl and hex from the objects
##########################################################################
stuff: ${INIT_OBJS} ${OBJS}


#  # Flash the board
#  ##########################################################################
#  flash:
#  	@echo "  PROG   $(OUTFILE).hex"
#  	@ps axc | grep lpc21isp >/dev/null; if [ $$? -eq 0 ] ; then killall lpc21isp ; fi
#  	@echo '$(LPC21ISP) $(LPC21ISP_OPTIONS) $(LPC21ISP_CONTROL) $(LPC21ISP_DEBUG) "$(CURDIR)/$(LPC21ISP_FLASHFILE)" $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)' > $(OBJFOLDER)/debugcommand.txt
#  	@chmod +x $(OBJFOLDER)/debugcommand.txt
#  	@open -a terminal.app $(OBJFOLDER)/debugcommand.txt
#  
#  
#  # Build the files and program the board
#  ##########################################################################
#  program:all flash
#  
#  
#  # Open a terminal using lpc21isp  This controls the bootloader so the
#  # board is not stuck in reset
#  ##########################################################################
#  term:
#  	@echo "TERM lpc21isp"
#  	@ps axc | grep lpc21isp >/dev/null; if [ $$? -eq 0 ] ; then killall lpc21isp ; fi		
#  	@echo '$(LPC21ISP) -termonly $(LPC21ISP_CONTROL) $(LPC21ISP_DEBUG) "$(CURDIR)/$(LPC21ISP_FLASHFILE)" $(LPC21ISP_PORT) $(LPC21ISP_BAUD) $(LPC21ISP_XTAL)' > debugcommand.txt
#  	@chmod +x debugcommand.txt
#  	@open -a terminal.app debugcommand.txt


# Print a variable
##########################################################################
print:
	@echo "INCLUDE DIRS $(INCLUDEDIRS)\n"
	@echo "OBJS $(OBJS)\n"
	@echo "VPATH $(VPATH)\n"


-include ${wildcard ${OBJFOLDER}/*.d} __dummy__





