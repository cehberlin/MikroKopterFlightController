#--------------------------------------------------------------------
# MCU name
MCU = atmega1284p
#MCU = atmega644p
F_CPU = 20000000
#-------------------------------------------------------------------
VERSION_MAJOR    =   2
VERSION_MINOR    =  12
VERSION_PATCH    =  0
VERSION_SERIAL_MAJOR = 11  	# Serial Protocol to KopterTool -> do not change!
VERSION_SERIAL_MINOR = 0  	# Serial Protocol
NC_SPI_COMPATIBLE = 76		# Navi-Kompatibilität
LIB_FC_COMPATIBLE = 7       # Library
#-------------------------------------------------------------------
# ATMEGA644: 63487 is maximum
#-------------------------------------------------------------------
# 0 a
# 1 b
# 2 c
# 3 d
# 4 e
# 5 f
# 6 g
# 7 h
# 8 i
# 9 j
# 10 k
# 11 L
#-------------------------------------------------------------------

# get SVN revision
REV := $(shell sh -c "cat .svn/entries | sed -n '4p'")

ifeq ($(MCU), atmega1284p)
FUSE_SETTINGS = -u -U lfuse:w:0xff:m -U hfuse:w:0xdf:m
 HEX_NAME = MEGA1284P
 LIBFC_EXT = 1284
endif

ifeq ($(MCU), atmega644p)
 FUSE_SETTINGS = -u -U lfuse:w:0xff:m -U hfuse:w:0xdf:m
 HEX_NAME = MEGA644
 LIBFC_EXT = 644
endif

ifeq ($(F_CPU), 16000000)
 QUARZ = 16MHZ
endif

ifeq ($(F_CPU), 20000000)
 QUARZ = 20MHZ
endif


# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# Target file name (without extension).

ifeq ($(VERSION_PATCH), 0)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)a_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 1)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)b_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 2)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)c_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 3)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)d_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 4)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)e_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 5)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)f_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 6)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)g_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 7)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)h_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 8)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)i_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 9)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)j_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 10)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)k_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 11)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)L_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 12)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)m_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 13)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)n_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 14)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)o_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 15)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)p_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 16)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)q_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 17)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)r_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 18)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)s_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 19)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)t_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 20)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)u_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 21)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)v_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 22)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)w_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 23)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)x_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 24)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)y_SVN$(REV)
endif
ifeq ($(VERSION_PATCH), 25)
TARGET = Flight-Ctrl_$(HEX_NAME)_V$(VERSION_MAJOR)_$(VERSION_MINOR)z_SVN$(REV)
endif


# Optimization level, can be [0, 1, 2, 3, s]. 0 turns off optimization.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s
#OPT = 2

##########################################################################################################
# List C source files here. (C dependencies are automatically generated.)
SRC = main.c uart.c timer0.c analog.c menu.c eeprom.c
SRC += twimaster.c rc.c fc.c GPS.c spi.c led.c Spektrum.c
SRC += mymath.c jetimenu.c capacity.c debug.c
SRC += hottmenu.c sbus.c user_receiver.c M-Link.c
SRC += jeti_ex.c
##########################################################################################################


# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = isqrt.S



# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = 


# Optional compiler flags.
#  -g:        generate debugging information (for GDB, or for COFF conversion)
#  -O*:       optimization level
#  -f...:     tuning, see gcc manual and avr-libc documentation
#  -Wall...:  warning level
#  -Wa,...:   tell GCC to pass this to the assembler.
#    -ahlms:  create assembler listing
CFLAGS = -O$(OPT) -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -Wa,-adhlns=$(<:%.c=%.lst) $(patsubst %,-I%,$(EXTRAINCDIRS))


# Set a "language standard" compiler flag.
#   Unremark just one line below to set the language standard to use.
#   gnu99 = C99 + GNU extensions. See GCC manual for more information.
#CFLAGS += -std=c89
#CFLAGS += -std=gnu89
#CFLAGS += -std=c99
CFLAGS += -std=gnu99

# shrink code size
CFLAGS += -mtiny-stack
#CFLAGS += -fno-inline-functions
CFLAGS += -mcall-prologues

CFLAGS += -DF_CPU=$(F_CPU) -DVERSION_MAJOR=$(VERSION_MAJOR) -DVERSION_MINOR=$(VERSION_MINOR) -DVERSION_PATCH=$(VERSION_PATCH) -DVERSION_SERIAL_MAJOR=$(VERSION_SERIAL_MAJOR) -DVERSION_SERIAL_MINOR=$(VERSION_SERIAL_MINOR) -DNC_SPI_COMPATIBLE=$(NC_SPI_COMPATIBLE) -DLIB_FC_COMPATIBLE=$(LIB_FC_COMPATIBLE)


# Optional assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 



# Optional linker flags.
#  -Wl,...:   tell GCC to pass this to linker.
#  -Map:      create map file
#  --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref

# Additional libraries

# Minimalistic printf version
#LDFLAGS += -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires -lm below)
#LDFLAGS += -Wl,-u,vfprintf -lprintf_flt

# -lm = math library
LDFLAGS += -lm

LDFLAGS += libfc$(LIBFC_EXT).a

##LDFLAGS += -T./linkerfile/avr5.x



# Programming support using avrdude. Settings and variables.

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
#AVRDUDE_PROGRAMMER = dt006
#AVRDUDE_PROGRAMMER = stk200
#AVRDUDE_PROGRAMMER = ponyser
AVRDUDE_PROGRAMMER = avrispv2
#falls Ponyser ausgewählt wird, muss sich unsere avrdude-Configdatei im Bin-Verzeichnis des Compilers befinden

#AVRDUDE_PORT = com1    # programmer connected to serial device
#AVRDUDE_PORT = lpt1    # programmer connected to parallel port
AVRDUDE_PORT = usb # programmer connected to USB

#AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex 
AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex $(FUSE_SETTINGS)
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

#avrdude -c avrispv2 -P usb -p m32 -U flash:w:blink.hex
AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE += -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
AVRDUDE_FLAGS += -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_FLAGS += -v -v

# ---------------------------------------------------------------------------
# Define directories, if needed.
DIRAVR = c:/winavr
DIRAVRBIN = $(DIRAVR)/bin
DIRAVRUTILS = $(DIRAVR)/utils/bin
DIRINC = .
DIRLIB = $(DIRAVR)/avr/lib


# Define programs and commands.
SHELL = sh

CC = avr-gcc

OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size

# Programming support using avrdude.
AVRDUDE = avrdude

REMOVE = rm -f
COPY = cp

HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -x -A $(TARGET).elf
LIMITS = $(SIZE) --mcu=$(MCU) -C $(TARGET).elf


# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:


# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
#ALL_CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I. $(CFLAGS)
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: begin gccversion sizebefore $(TARGET).elf $(TARGET).hex sizeafter finished end


# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo Size before:; $(ELFSIZE); $(HEXSIZE); $(LIMITS); echo; fi
sizeafter:
	@if [ -f $(TARGET).elf ]; then echo Size after:; $(ELFSIZE); $(HEXSIZE); $(LIMITS); echo; fi


# Display compiler version information.
gccversion : 
	@$(CC) --version


# Convert ELF to COFF for use in debugging / simulating in
# AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
	--change-section-address .data-0x800000 \
	--change-section-address .bss-0x800000 \
	--change-section-address .noinit-0x800000 \
	--change-section-address .eeprom-0x810000 


coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof




# Program the device.  
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)




# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	avr-nm -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)


# Compile: create object files from C source files.
%.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@


# Compile: create assembler files from C source files.
%.s : %.c
	$(CC) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files.
%.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@






# Target: clean project.
clean: begin clean_list finished end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) Flight-Ctrl_*.hex
	$(REMOVE) Flight-Ctrl_*.eep
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).cof
	$(REMOVE) Flight-Ctrl_*.elf
	$(REMOVE) Flight-Ctrl_*.map
	$(REMOVE) $(TARGET).obj
	$(REMOVE) $(TARGET).a90
	$(REMOVE) Flight-Ctrl_*.sym
	$(REMOVE) $(TARGET).lnk
	$(REMOVE) $(TARGET).lss
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) $(SRC:.c=.o)


# Automatically generate C source code dependencies. 
# (Code originally taken from the GNU make user manual and modified 
# (See README.txt Credits).)
#
# Note that this will work with sh (bash) and sed that is shipped with WinAVR
# (see the SHELL variable defined above).
# This may not work with other shells or other seds.
#
%.d: %.c
	set -e; $(CC) -MM $(ALL_CFLAGS) $< \
	| sed 's,\(.*\)\.o[ :]*,\1.o \1.d : ,g' > $@; \
	[ -s $@ ] || rm -f $@


# Remove the '-' if you want to see the dependency files generated.
-include $(SRC:%.c=%.d)



# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion coff extcoff \
	clean clean_list program

