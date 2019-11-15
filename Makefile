CXX = g++


include Makefile.locale
GUROBI_INCLUDEDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/include/
GUROBI_LIBDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/lib/

#-D_GLIBCXX_USE_CXX11_ABI=0
LIBS = -lgurobi_c++ -lgurobi80 -lm  -m64  -lprotobuf -w -pthread  -std=c++11

CXXFLAGS = -MMD -I . -I ./src/  -I /usr/local/include/ -I $(GUROBI_INCLUDEDIR) -g -O3 -std=c++11

LINK_FLAGS = -g -L ./ -L /usr/local/lib/ -L $(GUROBI_LIBDIR)

OBJS = ./src/sherlock.o ./src/network_computation.o ./src/gurobi_interface.o \
./src/configuration.o ./src/nodes.o ./src/computation_graph.o ./src/region_constraints.o \
./src/generate_constraints.o ./src/network_signatures.o  ./src/selective_binarization.o \
./src/parsing_onnx.o ./src/onnx.pb.o ./src/image_handler.o  ./main.o

DEPENDS = ${OBJECTS:.o=.d}


all: libs run_file

libs: $(OBJS)
	ar rcs ./src/libsherlock.a $(OBJS)
	ranlib ./src/libsherlock.a
	cp ./src/*.h ./include


run_file: main.o $(OBJS)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

%.o: %.cc
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.cpp
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.c
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f ./src/*.o *.o ./run_file ./lib/* ./include/*.h

-include ${DEPENDS}
