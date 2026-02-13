# ---------------------------------------------------------------------------- #
# VEX V5 VS Code extension makefile                                           #
# ---------------------------------------------------------------------------- #
# This is the standard top-level makefile expected by the VEX extension.
# It gathers C/C++ source files from src/ and include paths from include/.

SRC_C += $(wildcard src/*.c)
SRC_CPP += $(wildcard src/*.cpp)

INCLUDE += -Iinclude

# Pull in the VEX V5 build rules bundled with VEXcode / VS Code extension.
include vex/mkrules.mk
