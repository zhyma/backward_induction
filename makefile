CC = g++
NVCC = nvcc
CXXFLAGS = -std=c++11 -pthread -O3
LIBS = -lcudart
LIBDIRS = -L/usr/local/cuda/lib64
BUILD_DIR = ./build

GPU_SRC = gpu_solver.cu
GPU_OBJ = $(GPU_SRC:%.cu=$(BUILD_DIR)/%.o)
GPU_TARGET = $(GPU_SRC:%.cu=%)

CPU_SRC = $(wildcard *.cpp)
CPU_OBJ = $(CPU_SRC:%.cpp=$(BUILD_DIR)/%.o)
CPU_TARGET = $(CPU_SRC:%.cpp=%)

XML_SRC = ./tinyxml2/tinyxml2.cpp
XML_OBJ = ./build/tinyxml2.o

# objects = dp_solver

# edit:$(objects)

dp_solver: prepare cpu_module gpu_module xml_module $(CPU_OBJ) $(GPU_OBJ) $(XML_OBJ)
	$(NVCC) -dlink ./build/gpu_solver.o -o ./build/gpu.dlink.o
	$(CC) $(CXXFLAGS) $(CPU_OBJ) $(GPU_OBJ) $(XML_OBJ) ./build/gpu.dlink.o $(LIBDIRS) $(LIBS) -o ./build/dp_solver

prepare:
ifneq ($(wildcard build),)
	@echo "Found ./build."
else
	@echo "Did not find ./build, create a new one"
	mkdir -p $(BUILD_DIR)
endif
ifneq ($(wildcard output),)
	@echo "Found ./output."
else
	@echo "Did not find ./output, create a new one"
	mkdir -p output
endif
ifneq ($(wildcard fig),)
	@echo "Found ./fig."
else
	@echo "Did not find ./fig, create a new one"
	mkdir -p fig
endif

cpu_module: $(CPU_SRC)
	@for i in $(CPU_TARGET); do \
		echo "complile $$i.cpp";\
		$(CC) $(CXXFLAGS) -c $$i.cpp -o ${BUILD_DIR}/$$i.o; \
	done

gpu_module: $(GPU_SRC)
	@for i in $(GPU_TARGET); do \
		echo "complile $$i.cu";\
		$(NVCC) -dc $$i.cu -o ${BUILD_DIR}/$$i.o; \
	done

xml_module: $(XML_SRC)
	$(CC) $(CXXFLAGS) -c $(XML_SRC) -o $(XML_OBJ)

.PHONY:clean
clean:$(objects)
	-rm $(BUILD_DIR)/*
	-rm ./output/*
