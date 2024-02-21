################################################################################
# PROJECT makefile
################################################################################

# Create this file local settings
-include makefile.settings

# Build tool settings
RM := rm
ARM_CC := arm-none-eabi-gcc
SHARC_CC := cc21k
SHARC_AS := easm21k
LD := elfloader

# PROJECT settings
PROJECT := SAM-Audio-Starter

# Create this file for configuration specific settings
-include $(CONFIG)/makefile.include

# Project settings
PROC = ADSP-SC589
SI_REVISION = 1.0
LDR = $(PROJECT).ldr

# Set release/debug/optimizer flags
DEBUG ?= yes
RELEASE ?= no
ifeq ($(RELEASE), yes)
	ifeq ($(DEBUG), yes)
		BUILD_RELEASE = -g
	else
		BUILD_RELEASE =
	endif
	ARM_OPTIMIZE ?= -O1
	SHARC_OPTIMIZE ?= -O1 -Ov100 -Oa
else
	BUILD_RELEASE = -g
endif

SHARC0_OPTIMIZE ?= $(SHARC_OPTIMIZE)
SHARC1_OPTIMIZE ?= $(SHARC_OPTIMIZE)

################################################################################
# ARM section
################################################################################

# All of the ARM sources participating in the build are defined here
SRC_PREFIX = ..
ARM_SRC_DIRS += \
	ALL \
	ALL/src/sae \
	ARM \
	ARM/src \
	ARM/src/adi-drivers/rsi \
	ARM/src/simple-drivers \
	ARM/src/simple-services/adau1761 \
	ARM/src/simple-services/syslog \
	ARM/src/simple-services/buffer-track \
	ARM/src/simple-services/a2b-xml \
	ARM/src/simple-services/adi-a2b-cmdlist \
	ARM/src/simple-services/FreeRTOS-cpu-load \
	ARM/src/adi-drivers/ethernet/common \
	ARM/src/adi-drivers/ethernet/gemac \
	ARM/src/simple-services/adi-osal-minimal \
	ARM/src/simple-services/uac2-soundcard \
	ARM/src/simple-services/fs-dev \
	ARM/src/oss-services/lwip/core \
	ARM/src/oss-services/lwip/core/ipv4 \
	ARM/src/oss-services/lwip/netif \
	ARM/src/oss-services/lwip/arch \
	ARM/src/oss-services/lwip/api \
	ARM/src/simple-services/a2b-to-sport-cfg \
	ARM/src/oss-services/umm_malloc \
	ARM/src/oss-services/shell \
	ARM/src/oss-services/crc \
	ARM/src/oss-services/xmodem \
	ARM/src/oss-services/yxml \
	ARM/src/oss-services/pa-ringbuffer \
	ARM/src/oss-services/spiffs \
	ARM/src/oss-services/FreeRTOS-ARM \
	ARM/src/oss-services/FreeRTOS-ARM/portable/MemMang \
	ARM/src/oss-services/FreeRTOS-ARM/portable/CCES/ARM_CA5 \
	ARM/src/oss-services/FreeRTOS-ARM/portable/GCC/ARM_CA9 \
	ARM/src/oss-services/FatFs \
	ARM/src/oss-services/libtelnet \
	ARM/src/oss-services/kilo \
	ARM/src/oss-services/getline

ARM_DST := ARM
ARM_DST_DIRS := $(addprefix $(ARM_DST)/,$(ARM_SRC_DIRS))

# Include directories (SRC directories are automatically added)
ARM_INCLUDE_DIRS += \
	-I. \
	-I"../ALL/include" \
	-I"../ARM/include" \
	-I"../ARM/src/adi-drivers/ethernet/include" \
	-I"../ARM/src/oss-services/lwip" \
	-I"../ARM/src/oss-services/lwip/include" \
	-I"../ARM/src/oss-services/FreeRTOS-ARM/include"

# Add src directories to includes
ARM_INCLUDE_DIRS += $(addprefix -I$(SRC_PREFIX)/, $(ARM_SRC_DIRS))

ARM_C_SRC = $(foreach srcdir, $(ARM_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.c))
ARM_C_OBJ = $(subst $(SRC_PREFIX)/,$(ARM_DST)/,${ARM_C_SRC:%.c=%.o})

ARM_ASM_SRC = $(foreach srcdir, $(ARM_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.S))
ARM_ASM_OBJ = $(subst $(SRC_PREFIX)/,$(ARM_DST)/,${ARM_ASM_SRC:%.S=%.o})

# Can't do this automatically due to library linking order dependency.
# When the GNU linker sees a library, it discards all symbols that it
# doesn't need.
ARM_STATIC_LIBS = ../ARM/lib/cld_sc58x_audio_2_0_w_cdc_lib_Core0.a

# Setup dependencies
ARM_DEPS = $(foreach depdir, $(ARM_DST_DIRS), $(wildcard $(depdir)/*.d))

# define the compiler/assembler flags ARM_OPTIMIZE is meant to be passed in
# on the command-line (i.e. make ARM_OPTIMIZE=-O1)
ARM_CFLAGS = $(GENERAL_FLAGS)
ARM_CFLAGS += $(ARM_OPTIMIZE) $(BUILD_RELEASE) $(BUILD_RTOS) $(ARM_INCLUDE_DIRS)
ARM_CFLAGS += -Wall -Wno-unused-but-set-variable -Wno-unused-function
ARM_CFLAGS += -mcpu=cortex-a5 -gdwarf-2 -ffunction-sections -fdata-sections
ARM_CFLAGS += -mproc=$(PROC) -msi-revision=$(SI_REVISION) -DCORE0
ARM_CFLAGS += -DSAE_IPC -DFREE_RTOS -D__SAM_V1__ -DUSB_CDC_STDIO
# Uncomment for red boards (Warning: Ethernet broken)
#ARM_CFLAGS += -DRED_BOARD

ARM_AFLAGS = -x assembler-with-cpp -mproc=$(PROC) -msi-revision=$(SI_REVISION) -gdwarf-2 -DCORE0

ARM_OBJS = $(ARM_ASM_OBJ) $(ARM_C_OBJ)
ARM_OBJS_FILE = .lobjs_arm

ARM_EXE = $(PROJECT)-ARM.exe

ARM_CFLAGS_FILE = .cflags_arm
$(ARM_CFLAGS_FILE):
	echo $(ARM_CFLAGS) > $@

# Compile ARM 'C' files
$(ARM_DST)/%.o: ../%.c $(ARM_CFLAGS_FILE)
	$(ARM_CC) @$(ARM_CFLAGS_FILE) -I"$(SRC_PREFIX)/$(dir $@)" -MMD -MP -MF "$(basename $@).d" -o "$@" -c "$<"

# Compile ARM assembly files
$(ARM_DST)/%.o: ../%.S
	$(ARM_CC) $(ARM_AFLAGS) -MMD -MP -MF "$(basename $@).d" -o "$@" -c "$<"

# Create the ARM executable
$(ARM_EXE): $(ARM_OBJS) $(ARM_STATIC_LIBS)
	@echo $^ > $(ARM_OBJS_FILE)
	$(ARM_CC) -mproc=$(PROC) -msi-revision=$(SI_REVISION) -T"ARM-$(PROJECT).ld" -Wl,-Map=$(PROJECT)-ARM.map -Wl,--gc-sections -madi-threads -mdebug-libs -o "$@" @$(ARM_OBJS_FILE) -lm

# Use order-only-prerequisites for directories
$(ARM_OBJS): | $(ARM_DST_DIRS)

################################################################################
# SHARC0 section
################################################################################

# All of the SHARC0 sources participating in the build are defined here
SHARC0_SRC_DIRS += \
	ALL \
	ALL/src/sae \
	SHARC0 \
	SHARC0/src \
	SHARC0/startup_ldf

SHARC0_DST := SHARC0
SHARC0_DST_DIRS = $(addprefix $(SHARC0_DST)/,$(SHARC0_SRC_DIRS))

# Include directories (SRC dirs are automatically added)
SHARC0_INCLUDE_DIRS += \
	-I"../ALL/include" \
	-I"../SHARC0/include"

# Add src directories to includes
SHARC0_INCLUDE_DIRS += $(addprefix -I$(SRC_PREFIX)/, $(SHARC0_SRC_DIRS))

# Add SHARC0 static libs
SHARC0_STATIC_LIBS =

SHARC0_EXE = $(PROJECT)-SHARC0.exe

SHARC0_C_SRC = $(foreach srcdir, $(SHARC0_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.c))
SHARC0_C_OBJ = $(subst $(SRC_PREFIX)/,$(SHARC0_DST)/,${SHARC0_C_SRC:%.c=%.doj})

SHARC0_ASM_SRC = $(foreach srcdir, $(SHARC0_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.s))
SHARC0_ASM_OBJ = $(subst $(SRC_PREFIX)/,$(SHARC0_DST)/,${SHARC0_ASM_SRC:%.s=%.doj})

# Setup dependencies
SHARC0_DEPS = $(foreach depdir, $(SHARC0_DST_DIRS), $(wildcard $(depdir)/*.d))

# Define the compiler/assembler flags
SHARC0_CFLAGS += $(GENERAL_FLAGS) $(SHARC_CFLAGS)
SHARC0_CFLAGS += $(SHARC0_OPTIMIZE) $(BUILD_RELEASE) -proc $(PROC) $(SHARC0_INCLUDE_DIRS) -si-revision $(SI_REVISION) -DCORE1 -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -char-size-8 -swc
SHARC0_AFLAGS = $(BUILD_RELEASE) -proc $(PROC) -si-revision $(SI_REVISION) -DCORE1 -swc -char-size-8

SHARC0_OBJS = $(SHARC0_ASM_OBJ) $(SHARC0_C_OBJ)
SHARC0_OBJS_FILE = .lobjs_sharc0

SHARC0_EXE = $(PROJECT)-SHARC0.dxe

SHARC0_CFLAGS_FILE = .cflags_sharc0
$(SHARC0_CFLAGS_FILE):
	echo $(SHARC0_CFLAGS) > $@

# Compile SHARC0 'C' files
$(SHARC0_DST)/%.doj: ../%.c $(SHARC0_CFLAGS_FILE)
	$(SHARC_CC) -@ $(SHARC0_CFLAGS_FILE) -I"$(SRC_PREFIX)/$(dir $@)" -gnu-style-dependencies -MD -Mo "$(basename $@).d" -o "$@" -c "$<"

# Compile SHARC0 assembly files
$(SHARC0_DST)/SHARC0/%.doj: ../SHARC0/%.s
	$(SHARC_AS) $(SHARC0_AFLAGS) -gnu-style-dependencies -MM -Mo "$(basename $@).d" -o "$@" "$<"

# Link object files
$(SHARC0_EXE): $(SHARC0_OBJS) $(SHARC0_STATIC_LIBS)
	@echo $^ > $(SHARC0_OBJS_FILE)
	$(SHARC_CC) -proc $(PROC) -si-revision $(SI_REVISION) -T"SHARC0-$(PROJECT).ldf" -flags-link -e -no-mem -map linker_log_SHARC0.xml -flags-link -ip -flags-link -MDCORE1,-MDOTHERCORE,-MDDEBUG -o "$@" -@$(SHARC0_OBJS_FILE)
	@$(RM) -f $@.doj
	@$(RM) -f $@.asm

# Use order-only-prerequisites for directories
$(SHARC0_OBJS): | $(SHARC0_DST_DIRS)

################################################################################
# SHARC1 section
################################################################################

# All of the SHARC0 sources participating in the build are defined here
SHARC1_SRC_DIRS += \
	ALL \
	ALL/src/sae \
	SHARC1 \
	SHARC1/src \
	SHARC1/startup_ldf

SHARC1_DST := SHARC1
SHARC1_DST_DIRS = $(addprefix $(SHARC1_DST)/,$(SHARC1_SRC_DIRS))

# Include directories
SHARC1_INCLUDE_DIRS += \
	-I../ALL/include \
	-I../SHARC1/include

SHARC1_INCLUDE_DIRS += $(addprefix -I$(SRC_PREFIX)/, $(SHARC1_SRC_DIRS))

SHARC1_STATIC_LIBS =

SHARC1_EXE = $(PROJECT)-SHARC1.exe

SHARC1_C_SRC = $(foreach srcdir, $(SHARC1_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.c))
SHARC1_C_OBJ = $(subst $(SRC_PREFIX)/,$(SHARC1_DST)/,${SHARC1_C_SRC:%.c=%.doj})

SHARC1_ASM_SRC = $(foreach srcdir, $(SHARC1_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.s))
SHARC1_ASM_OBJ = $(subst $(SRC_PREFIX)/,$(SHARC1_DST)/,${SHARC1_ASM_SRC:%.s=%.doj})

# Setup dependencies
SHARC1_DEPS = $(foreach depdir, $(SHARC1_DST_DIRS), $(wildcard $(depdir)/*.d))

# Define the compiler/assembler flags
SHARC1_CFLAGS += $(GENERAL_FLAGS) $(SHARC_CFLAGS)
SHARC1_CFLAGS += $(SHARC1_OPTIMIZE) $(BUILD_RELEASE) -proc $(PROC) $(SHARC1_INCLUDE_DIRS) -si-revision $(SI_REVISION) -DCORE2 -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos -double-size-32 -char-size-8 -swc
SHARC1_AFLAGS = $(BUILD_RELEASE) -proc $(PROC) -si-revision $(SI_REVISION) -DCORE2 -swc -char-size-8

SHARC1_CFLAGS_FILE = .cflags_sharc1
$(SHARC1_CFLAGS_FILE):
	echo $(SHARC1_CFLAGS) > $@

SHARC1_OBJS = $(SHARC1_ASM_OBJ) $(SHARC1_C_OBJ)
SHARC1_OBJS_FILE = .lobjs_sharc1

SHARC1_EXE = $(PROJECT)-SHARC1.dxe

# Compile SHARC1 'C' files
$(SHARC1_DST)/%.doj: ../%.c $(SHARC1_CFLAGS_FILE)
	$(SHARC_CC) -@$(SHARC1_CFLAGS_FILE) -I"$(SRC_PREFIX)/$(dir $@)" -gnu-style-dependencies -MD -Mo "$(basename $@).d" -o "$@" -c "$<"

# Compile SHARC1 assembly files
$(SHARC1_DST)/%.doj: ../%.s
	$(SHARC_AS) $(SHARC1_AFLAGS) -gnu-style-dependencies -MM -Mo "$(basename $@).d" -o "$@" "$<"

# Link object files
$(SHARC1_EXE): $(SHARC1_OBJS) $(SHARC1_STATIC_LIBS)
	@echo $^ > $(SHARC1_OBJS_FILE)
	$(SHARC_CC) -proc $(PROC) -si-revision $(SI_REVISION) -T"SHARC1-$(PROJECT).ldf" -no-mem -map linker_log_SHARC1.xml -flags-link -e -flags-link -ip -flags-link -MDCORE2,-MDOTHERCORE,-MDDEBUG -o "$@" -@$(SHARC1_OBJS_FILE)
	@$(RM) -f $@.doj
	@$(RM) -f $@.asm

# Use order-only-prerequisites for directories
$(SHARC1_OBJS): | $(SHARC1_DST_DIRS)

################################################################################
# Generic section
################################################################################

.DEFAULT_GOAL = all
all: $(LDR)

$(LDR):  $(ARM_EXE) $(SHARC0_EXE) $(SHARC1_EXE)
	$(LD) -proc $(PROC) -core0=$(ARM_EXE) -init ezkitSC589_initcode_core0_v10 -core1=$(SHARC0_EXE) -core2=$(SHARC1_EXE) -NoFinalTag=$(ARM_EXE) -NoFinalTag=$(SHARC0_EXE) -b SPI -f BINARY -Width 8 -bcode 0x0C -o $@

$(ARM_DST_DIRS) $(SHARC0_DST_DIRS) $(SHARC1_DST_DIRS):
	mkdir -p $@

# Other Targets
clean:
	$(RM) -f *.xml $(PROJECT).* ../*.xml *.map
	$(RM) -rf $(ARM_EXE) $(ARM_DST)
	$(RM) -rf $(SHARC0_EXE) $(SHARC0_DST)
	$(RM) -rf $(SHARC1_EXE) $(SHARC1_DST)
	$(RM) -f ${LDR}
	$(RM) -f $(ARM_CFLAGS_FILE) $(SHARC0_CFLAGS_FILE) $(SHARC1_CFLAGS_FILE)
	$(RM) -f $(ARM_OBJS_FILE) $(SHARC0_OBJS_FILE) $(SHARC1_OBJS_FILE)

# Additional targets depending on configuration
-include $(CONFIG)/makefile.targets

help:
	@echo 'usage:'
	@echo '    make [all|clean] [ARM_OPTIMIZE=<-Og,-O1,etc.>] [RELEASE=<yes/no>] [VERBOSE=1]'
	@echo ''
	@echo 'DEBUG:'
	@echo '    make all ARM_OPTIMIZE=-Og SHARC_OPTIMIZE=-O1'
	@echo ''
	@echo 'RELEASE:'
	@echo '    make all'


.PHONY: all clean help
.SECONDARY:

# Include dependencies
-include $(ARM_DEPS)
-include $(SHARC0_DEPS)
-include $(SHARC1_DEPS)
