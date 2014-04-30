CC = avr-gcc.exe
OBJCOPY = avr-objcopy.exe
CFLAGS = -O2 -mmcu=atmega328p -Wall -pedantic -DF_CPU=16000000
LINKER_FLAGS = -lm -lprintf_flt -lscanf_flt
BUILD_DIR = build

target = goalie

sources = \
  adc.c \
  ramp.c \
  steppercontrol.c \
  uart_mega8.c \

objects = $(patsubst %.c,$(BUILD_DIR)/%.o, $(sources))
  
vpath %.c .
vpath %.o $(BUILD_DIR)
 
all: $(BUILD_DIR) $(BUILD_DIR)/$(target).hex

.PHONY:
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

$(BUILD_DIR)/$(target).hex: $(BUILD_DIR)/$(target).elf
	$(OBJCOPY) -O ihex $^ $@
	avr-size $@
	
$(BUILD_DIR)/$(target).elf : $(objects)
	$(CC) $^ $(LINKER_FLAGS) -o $@

$(BUILD_DIR)/%.o: %.c
	$(CC) $< -c $(CFLAGS) -o $@
	
clean:
	rm $(BUILD_DIR)/$(target).hex $(BUILD_DIR)/$(target).elf $(objects)