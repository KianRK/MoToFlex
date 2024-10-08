# C++ Console Executable Makefile autogenerated by premake
# Don't edit this file! Instead edit `premake.lua` then rerun `make`
# No, that is not true anymore as premake is totally outdated
# Example call: make CONFIG=Release DSGUI=1 QUIET=1
ifndef CONFIG
  CONFIG=Debug
endif
  
ifdef DSGUI
  DS_LIB = -L"../tools/ode-0.16.4/drawstuff/src/.libs/" -ldrawstuff
else
  CFLAGS += -DHEADLESS
endif

ifdef QUIET
  CFLAGS += -DSIM_QUIET
endif

# Please specify Python version via arg if different, e.g. make PYVER=python3.8
ifndef PYVER
  PYVER := python3.11
endif

# if multiple archs are defined turn off automated dependency generation
DEPFLAGS := $(if $(word 2, $(TARGET_ARCH)), , -MMD)

# Change version if required
PYTHON_CONFIG := $(PYVER)-config
PYTHON_LIB := $(PYVER)
ODE_LIB := ../tools/ode-0.16.4/ode/src/.libs/


PYTHONINCLUDE := $(shell $(PYTHON_CONFIG) --includes)
PYTHONLIBPATH := $(shell $(PYTHON_CONFIG) --ldflags)
PYTHONLIBS := $(shell $(PYTHON_CONFIG) --libs)

ifeq ($(OS),Windows_NT)

else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
		LBITS := $(shell getconf LONG_BIT)
		ifeq ($(LBITS),64)
			CFLAGS += -DLINUX64
		endif
		# On Linux additional flags are required to build libode.a: ./configure --with-pic --disable-demos --disable-asserts
		LDFLAGS += -L$(BINDIR) -L$(LIBDIR) $(PYTHONLIBPATH) $(PYTHONLIBS) -l$(PYTHON_LIB) -L$(ODE_LIB) -lode $(DS_LIB) -lpthread
    endif
    ifeq ($(UNAME_S),Darwin)
        LDFLAGS += -L$(BINDIR) -L$(LIBDIR) $(PYTHONLIBPATH) $(PYTHONLIBS) -l$(PYTHON_LIB) -L/opt/homebrew/lib -L"../ode/lib" -lode $(DS_LIB) -framework OpenGL -framework GLUT
    endif
endif

ifeq ($(CONFIG),Debug)
  OPTIMIZE_FLAGS = -g
endif

ifeq ($(CONFIG),Release)
  OPTIMIZE_FLAGS = -O2
endif

BINDIR := bin
LIBDIR := .
OBJDIR := obj/Debug
OUTDIR := bin
PYTHONPKGDIR := motoflex_gym
CPPFLAGS := $(DEPFLAGS) -I$(PYTHONINCLUDE) -I"../ode/include" -I"../shared" -DWALKING_SIMULATOR -I"." -I"../DortmundWalkingEngine"
CFLAGS += $(CPPFLAGS) $(TARGET_ARCH) $(OPTIMIZE_FLAGS) -I"../ode/include" -fPIC
CXXFLAGS += $(CFLAGS)
LDDEPS :=
RESFLAGS := -I "../ode/include"
EXE := WalkingSimulator
SO := WalkingSimulator.so
BLDCMD = $(CXX) -o $(OUTDIR)/$(EXE) $(OBJECTS) $(LDFLAGS) $(RESOURCES) $(TARGET_ARCH)
SOCMD = $(CXX) -o $(PYTHONPKGDIR)/$(SO) $(OBJECTS) $(LDFLAGS) -shared -pthread -fPIC -fwrapv $(RESOURCES) $(TARGET_ARCH)


OBJECTS := \
	$(OBJDIR)/csvparser.o \
	$(OBJDIR)/FastFilter.o \
	$(OBJDIR)/Filter.o \
	$(OBJDIR)/nao.o \
	$(OBJDIR)/behaviour.o \
	$(OBJDIR)/box.o \
	$(OBJDIR)/doublejoint.o \
	$(OBJDIR)/flexiblebox.o \
	$(OBJDIR)/joint.o \
	$(OBJDIR)/motor.o \
	$(OBJDIR)/parameters.o \
	$(OBJDIR)/pidcontroller.o \
	$(OBJDIR)/simulation.o \
	$(OBJDIR)/singlejoint.o \
	$(OBJDIR)/anglefitness.o \
	$(OBJDIR)/anglefromcsv.o \
	$(OBJDIR)/anglereader.o \
	$(OBJDIR)/Matrix.o \
	$(OBJDIR)/genom.o \
	$(OBJDIR)/paths.o \
	$(OBJDIR)/world.o \
	$(OBJDIR)/config.o \
	$(OBJDIR)/MVTools.o \
	$(OBJDIR)/csvlogger.o \
	$(OBJDIR)/pythonmodule.o \
	$(OBJDIR)/anglefrompython.o \
	$(OBJDIR)/kinematics.o

MKDIR_TYPE := msdos
CMD := $(subst \,\\,$(ComSpec)$(COMSPEC))
ifeq (,$(CMD))
  MKDIR_TYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  MKDIR_TYPE := posix
endif
ifeq ($(MKDIR_TYPE),posix)
  CMD_MKBINDIR := mkdir -p $(BINDIR)
  CMD_MKLIBDIR := mkdir -p $(LIBDIR)
  CMD_MKOUTDIR := mkdir -p $(OUTDIR)
  CMD_MKOBJDIR := mkdir -p $(OBJDIR)
else
  CMD_MKBINDIR := $(CMD) /c if not exist $(subst /,\\,$(BINDIR)) mkdir $(subst /,\\,$(BINDIR))
  CMD_MKLIBDIR := $(CMD) /c if not exist $(subst /,\\,$(LIBDIR)) mkdir $(subst /,\\,$(LIBDIR))
  CMD_MKOUTDIR := $(CMD) /c if not exist $(subst /,\\,$(OUTDIR)) mkdir $(subst /,\\,$(OUTDIR))
  CMD_MKOBJDIR := $(CMD) /c if not exist $(subst /,\\,$(OBJDIR)) mkdir $(subst /,\\,$(OBJDIR))
endif

.PHONY: clean all

all: $(OUTDIR)/$(EXE) $(PYTHONPKGDIR)/$(SO)

$(OUTDIR)/$(EXE): $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking $(EXE)
	-@$(CMD_MKBINDIR)
	-@$(CMD_MKLIBDIR)
	-@$(CMD_MKOUTDIR)
	@$(BLDCMD)

$(PYTHONPKGDIR)/$(SO): $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking $(SO)
	-@$(CMD_MKBINDIR)
	-@$(CMD_MKLIBDIR)
	-@$(CMD_MKOUTDIR)
	@$(SOCMD)

clean:
	@echo Cleaning WalkingSimulator
ifeq ($(MKDIR_TYPE),posix)
	-@rm -f $(OUTDIR)/$(TARGET)
	-@rm -rf $(OBJDIR)
else
	-@if exist $(subst /,\,$(OUTDIR)/$(TARGET)) del /q $(subst /,\,$(OUTDIR)/$(TARGET))
	-@if exist $(subst /,\,$(OBJDIR)) del /q $(subst /,\,$(OBJDIR))
	-@if exist $(subst /,\,$(OBJDIR)) rmdir /s /q $(subst /,\,$(OBJDIR))
endif

$(OBJDIR)/anglefromcsv.o: adapter/anglefromcsv.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/anglefrompython.o: adapter/anglefrompython.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/anglefitness.o: ea/anglefitness.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/csvparser.o: csvclass/csvparser.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/FastFilter.o: filter/FastFilter.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/Filter.o: filter/Filter.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/nao.o: nao/nao.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/behaviour.o: simulation/behaviour.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/box.o: simulation/box.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/doublejoint.o: simulation/doublejoint.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/flexiblebox.o: simulation/flexiblebox.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/joint.o: simulation/joint.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/motor.o: simulation/motor.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/parameters.o: simulation/parameters.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/pidcontroller.o: simulation/pidcontroller.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/simulation.o: simulation/simulation.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/singlejoint.o: simulation/singlejoint.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/anglereader.o: tools/anglereader.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/Matrix.o: ../shared/math/Matrix.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/genom.o: ../shared/genom.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/paths.o: params/paths.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/world.o: simulation/world.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/config.o: ../shared/config.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/MVTools.o: ../DortmundWalkingEngine/Tools/Math/MVTools.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/csvlogger.o: ../shared/csvlogger.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/pythonmodule.o: motoflex_gym/pythonmodule.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

$(OBJDIR)/kinematics.o: tools/kinematics.cpp
	-@$(CMD_MKOBJDIR)
	@echo $(notdir $<)
	@$(CXX) $(CXXFLAGS) -o "$@" -c "$<"

-include $(OBJECTS:%.o=%.d)

