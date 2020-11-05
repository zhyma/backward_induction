objects=linear_func

edit:$(objects)

linear_func:
	nvcc -dc gpu_solver.cu -o ./build/gpu_solver.o
	nvcc -dlink ./build/gpu_solver.o -o ./build/gpu.dlink.o
	g++ -c phy_model.cpp -o ./build/phy_model.o
	g++ -c prob_tool.cpp -o ./build/prob_tool.o
	g++ -c dp_model.cpp -o ./build/dp_model.o
	g++ -c cpu_solver.cpp -o ./build/cpu_solver.o
	g++ -c main.cpp -o ./build/main.o
	g++ ./build/main.o ./build/dp_model.o ./build/cpu_solver.o ./build/prob_tool.o ./build/gpu.dlink.o ./build/gpu_solver.o ./build/phy_model.o -o ./build/test -L/usr/local/cuda/lib64 -lcudart


.PHONY:clean
clean:$(objects)
	-rm ./build/$(objects)
