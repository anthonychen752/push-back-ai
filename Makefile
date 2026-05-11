# VEXcode Makefile
# Robot Configuration: VEX V5
# Compiler: GCC 9.2.1 20191125 (or newer)

PROJECT = push-back

SRCFILES = $(wildcard src/*.cpp)
INCFILES = $(wildcard include/*.h)
OBJFILES = $(SRCFILES:.cpp=.o)

ARCHIVE = lib$(PROJECT).a
BINDIR = bin

CXX = arm-none-eabi-c++
CXXFLAGS = -Wall -O2 -mfloat-abi=soft -mcpu=cortex-a9 -fno-builtin -ffunction-sections -fsigned-char -fno-rtti -fno-exceptions -std=c++17 -Wno-register
LDFLAGS = -T/usr/../.v5/vexv5-arm.ld -Wl,-Map=$(BINDIR)/$(PROJECT).map,--gc-sections -lm

VEX_INC = /usr/../include/vex
VEX_LIBS = -lvex -lm

.PHONY: all clean

all: $(BINDIR)/$(PROJECT).elf

$(BINDIR)/%.o: src/%.cpp $(INCFILES) | $(BINDIR)
	$(CXX) $(CXXFLAGS) -I$(VEX_INC) -I./include -c $< -o $@

$(BINDIR)/$(PROJECT).elf: $(OBJFILES)
	$(CXX) $(OBJFILES) $(LDFLAGS) $(VEX_LIBS) -o $@

$(BINDIR):
	mkdir -p $(BINDIR)

clean:
	rm -rf $(BINDIR)

# Debug
debug: $(BINDIR)/$(PROJECT).elf
	arm-none-eabi-objdump -d $(BINDIR)/$(PROJECT).elf > $(BINDIR)/$(PROJECT).lst
