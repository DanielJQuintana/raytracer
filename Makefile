# GPP = g++
# FLAGS = -g -Wall -D_REENTRANT -std=c++0x -pthread

# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

.PHONY = all clean

CC = g++			# compiler to use

LIB_PATH = lib

# Input Names
CUDA_FILES_A = prm_gpu.cu 
CPP_FILES_A = planarutils.cpp vandercorput.cpp # astar.cpp
CPP_MAIN_A = prm_3D.cpp

CUDA_FILES_B = visualize.cu
CPP_FILES_B = glad.c
CPP_MAIN_B = visualize.cpp
CPP_DIS_B = display.cpp

# Directory names
SRCDIR = src
OBJDIR = obj
BINDIR = bin

#C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.0\include

CUDA_PATH = /usr/local/cuda
CUDA_INC_PATH = $(CUDA_PATH)/include
CUDA_BIN_PATH = $(CUDA_PATH)/bin
CUDA_LIB_PATH = $(CUDA_PATH)/lib64

NVCC = $(CUDA_BIN_PATH)/nvcc

# OS-architecture specific flags
ifeq ($(OS_SIZE),32)
	NVCC_FLAGS := -m32
else
	NVCC_FLAGS := -m64
endif

NVCC_FLAGS += -g -dc -Wno-deprecated-gpu-targets --std=c++11 \
				--expt-relaxed-constexpr -D THRUST_IGNORE_DEPRECATED_CPP_DIALECT
NVCC_INCLUDE = -Iinclude
NVCC_CUDA_LIBS = 
NVCC_GENCODES = -gencode arch=compute_50,code=sm_50 \
		-gencode arch=compute_52,code=sm_52 \
		-gencode arch=compute_60,code=sm_60 \
		-gencode arch=compute_61,code=sm_61


# ------------------------------------------------------------------------------

# CUDA Linker and Flags
CUDA_LINK_FLAGS = -dlink -Wno-deprecated-gpu-targets

# ------------------------------------------------------------------------------

# C++ Compiler and Flags
GPP = g++
FLAGS = -g -Wall -D_REENTRANT -std=c++11 -pthread
INCLUDE = -I$(CUDA_INC_PATH) -Iinclude
CUDA_LIBS = -L$(CUDA_LIB_PATH) -lcudart -lcufft -lcublas -lcudnn -lcurand
LIBS = -L$(LIB_PATH) -lglfw
RUNTIME_LIBS = -Wl,-rpath=${LIB_PATH}

# ------------------------------------------------------------------------------
# Object files
# ------------------------------------------------------------------------------

# CUDA Object Files
CUDA_OBJ_A = $(OBJDIR)/cuda_a.o
CUDA_OBJ_FILES_A = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CUDA_FILES_A)))
CUDA_OBJ_B = $(OBJDIR)/cuda_b.o
CUDA_OBJ_FILES_B = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CUDA_FILES_B)))

# C++ Object Files
CPP_OBJ_A = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CPP_FILES_A)))
PRM_OBJ_A = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CPP_MAIN_A)))

CPP_OBJ_B = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CPP_FILES_B)))
VIS_OBJ_B = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CPP_MAIN_B)))
DIS_OBJ_B = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(CPP_DIS_B)))

# List of all common objects needed to be linked into the final executable
COMMON_OBJ_A = $(CPP_OBJ_A) $(CUDA_OBJ_A) $(CUDA_OBJ_FILES_A)
COMMON_OBJ_B = $(CPP_OBJ_B) $(CUDA_OBJ_B) $(CUDA_OBJ_FILES_B)

# ------------------------------------------------------------------------------
# Make rules
# ------------------------------------------------------------------------------

# Top level rules
all: visualize visualize-cpu display prm-test prm-test-cpu

prm-test: $(COMMON_OBJ_A)
	$(GPP) $(FLAGS) -o $(BINDIR)/$@ $(INCLUDE) $^ $(CUDA_LIBS) 

prm-test-cpu: $(PRM_OBJ_A) $(CPP_OBJ_A)
	$(GPP) $(FLAGS) -o $(BINDIR)/$@ $(INCLUDE) $^

display: $(DIS_OBJ_B) $(CPP_OBJ_B)
	$(GPP) $(FLAGS) -o $(BINDIR)/$@ $(INCLUDE) $^ $(LIBS) $(RUNTIME_LIBS)

visualize: $(COMMON_OBJ_B)
	$(GPP) $(FLAGS) -o $(BINDIR)/$@ $(INCLUDE) $^ $(LIBS) $(RUNTIME_LIBS) $(CUDA_LIBS) 

visualize-cpu: $(VIS_OBJ_B) $(CPP_OBJ_B)
	$(GPP) $(FLAGS) -o $(BINDIR)/$@ $(INCLUDE) $^ $(LIBS) $(RUNTIME_LIBS)

# Compile C++ Source Files
$(PRM_OBJ_A): $(addprefix $(SRCDIR)/, $(CPP_MAIN_A))
	$(GPP) $(FLAGS) -c -o $@ $(INCLUDE) $< 

$(CPP_OBJ_A): $(OBJDIR)/%.o : $(SRCDIR)/%	
	$(GPP) $(FLAGS) -c -o $@ $(INCLUDE) $<

$(VIS_OBJ_B): $(addprefix $(SRCDIR)/,	$(CPP_MAIN_B))	
	$(GPP) $(FLAGS) -c -o $@ $(INCLUDE) $< 

$(DIS_OBJ_B): $(addprefix $(SRCDIR)/,	$(CPP_DIS_B))	
	$(GPP) $(FLAGS) -c -o $@ $(INCLUDE) $< 

$(CPP_OBJ_B): $(OBJDIR)/%.o : $(SRCDIR)/%	
	$(GPP) $(FLAGS) -c -o $@ $(INCLUDE) $<


# Compile CUDA Source Files
$(CUDA_OBJ_FILES_A): $(OBJDIR)/%.cu.o : $(SRCDIR)/%.cu
	$(NVCC) $(NVCC_FLAGS) $(NVCC_GENCODES) -c -o $@ $(NVCC_INCLUDE) $<

$(CUDA_OBJ_FILES_B): $(OBJDIR)/%.cu.o : $(SRCDIR)/%.cu
	$(NVCC) $(NVCC_FLAGS) $(NVCC_GENCODES) -c -o $@ $(NVCC_INCLUDE) $<

# # Make linked device code
$(CUDA_OBJ_A): $(CUDA_OBJ_FILES_A)
	$(NVCC) $(CUDA_LINK_FLAGS) $(NVCC_GENCODES) -o $@ $(NVCC_INCLUDE) $<

$(CUDA_OBJ_B): $(CUDA_OBJ_FILES_B)
	$(NVCC) $(CUDA_LINK_FLAGS) $(NVCC_GENCODES) -o $@ $(NVCC_INCLUDE) $<

# Clean everything including temporary Emacs files
clean:
	rm -f *.o $(BINDIR)/* $(OBJDIR)/*.o $(SRCDIR)/*~ *~
	cp -a lib/. bin/
