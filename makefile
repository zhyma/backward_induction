CC = g++
NVCC = nvcc
CXXFLAGS = -std=c++11 -pthread
LIBS = -lcudart
LIBDIRS = -L/usr/local/cuda/lib64
BUILD_DIR = ./build

GPU_SRC = gpu_solver.cu
GPU_OBJ = $(GPU_SRC:%.cu=$(BUILD_DIR)/%.o)
GPU_TARGET = $(GPU_SRC:%.cu=%)

CPU_SRC = $(wildcard *.cpp)
CPU_OBJ = $(CPU_SRC:%.cpp=$(BUILD_DIR)/%.o)
CPU_TARGET = $(CPU_SRC:%.cpp=%)

# objects = dp_solver

# edit:$(objects)

dp_solver: cpu_module gpu_module $(CPU_OBJ) $(GPU_OBJ)
	$(NVCC) -dlink ./build/gpu_solver.o -o ./build/gpu.dlink.o
	$(CC) $(CXXFLAGS) $(CPU_OBJ) $(GPU_OBJ) ./build/gpu.dlink.o $(LIBDIRS) $(LIBS) -o ./build/dp_solver

cpu_module: $(CPU_SRC)
	mkdir -p $(BUILD_DIR)
	@for i in $(CPU_TARGET); do \
		echo "complile $$i.cpp";\
		$(CC) $(CXXFLAGS) -c $$i.cpp -o ${BUILD_DIR}/$$i.o; \
    done

gpu_module: $(GPU_SRC)
	mkdir -p $(BUILD_DIR)
	@for i in $(GPU_TARGET); do \
		echo "complile $$i.cu";\
		$(NVCC) -dc $$i.cu -o ${BUILD_DIR}/$$i.o; \
    done

.PHONY:clean
clean:$(objects)
	-rm $(BUILD_DIR)/*
