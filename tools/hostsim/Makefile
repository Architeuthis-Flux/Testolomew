# The host programs: the simulator (sim), the soak and pencil benches, the
# nav stick decoder test. The firmware's own sources are compiled here
# against the stubs in this folder (Arduino.h, the LCD, the LED strip, the
# other core's sampler, EEPROM); GFX comes from PlatformIO's libdeps, so run
# `pio run` once first.
#
#   make sim        the simulator
#   make check      the regression scenes (scenes/check.txt, scenes/strength.txt)
#   make screens    every screen as a PNG under out/, and the montage in docs/
#   make soak pencil navtest
#   make clean

S = ../../src
G = ../../.pio/libdeps/testolomew/Adafruit GFX Library
B = ../../.pio/libdeps/testolomew/Adafruit BusIO
CXX ?= g++
CXXFLAGS ?= -std=c++11 -O1 -g -w -DARDUINO=10808 -MMD -MP
INC = -I . -I $(S) -I $(S)/common -I $(S)/jos -I $(S)/console -I $(S)/magarray -I $(S)/magfit -I $(S)/rowcount -I $(S)/display -I $(S)/probeled -I $(S)/ui -I $(S)/play -I $(S)/settings -I $(S)/apps -I $(S)/board -I "$(G)" -I "$(B)"

# Every firmware source but the ones the stubs replace.
DEVICE_ONLY = $(S)/board/BoardPins.cpp $(S)/display/ST7789.cpp $(S)/console/SerialDma.cpp $(S)/magarray/MagSampler.cpp $(S)/probeled/LedStrip.cpp $(S)/probeled/LedStripDark.cpp
FW_SRC = $(filter-out $(DEVICE_ONLY), $(wildcard $(S)/*.cpp $(S)/*/*.cpp))
STUB_SRC = SimGlobals.cpp LedStripStub.cpp MagSamplerStub.cpp St7789Stub.cpp
O = build
FW_OBJ = $(patsubst $(S)/%.cpp,$(O)/fw/%.o,$(FW_SRC))
STUB_OBJ = $(patsubst %.cpp,$(O)/%.o,$(STUB_SRC))
GFX_OBJ = $(O)/gfx/Adafruit_GFX.o
COMMON = $(FW_OBJ) $(STUB_OBJ) $(GFX_OBJ)

all: sim

sim: $(O)/sim.o $(COMMON)
	$(CXX) $(CXXFLAGS) $^ -o $@

soak: $(O)/soak.o $(COMMON)
	$(CXX) $(CXXFLAGS) $^ -o $@

pencil: $(O)/pencil.o $(COMMON)
	$(CXX) $(CXXFLAGS) $^ -o $@

navtest: $(O)/navtest.o $(COMMON)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(O)/fw/%.o: $(S)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INC) -c $< -o $@

# The GFX path has spaces: escaped for make's prerequisite list.
space :=
space +=
G_ESC = $(subst $(space),\$(space),$(G))
$(O)/gfx/Adafruit_GFX.o: $(G_ESC)/Adafruit_GFX.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INC) -c "$(G)/Adafruit_GFX.cpp" -o $@

$(O)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INC) -c $< -o $@

check: sim
	./sim scenes/check.txt -q
	./sim scenes/strength.txt -q

screens: sim
	rm -rf out
	mkdir -p out
	./sim scenes/screens.txt -q
	python3 montage.py ../../docs/screens-simulated.png $$(ls out/s*.png | sort)

clean:
	rm -rf $(O) sim soak pencil navtest out

-include $(O)/*.d $(O)/fw/*.d $(O)/fw/*/*.d $(O)/gfx/*.d

.PHONY: all check screens clean
