CXX = g++


include Makefile.locale
GUROBI_INCLUDEDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/include/
GUROBI_LIBDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/lib/

#-D_GLIBCXX_USE_CXX11_ABI=0
LIBS = -lgurobi_c++ -lgurobi80 -lflowstar -lmpfr -lgmp -lgsl -lgslcblas -lm -lglpk -lmpfi \
 -m64 -lprotobuf -w -pthread  -std=c++11

CXXFLAGS = -MMD -I . -I ./src/ -I ./src/neural_rule_analysis -I ./eigen_file/  -I /usr/local/include/ \
-I $(GUROBI_INCLUDEDIR) -I ./flowstar-release/ -g -O3 -std=c++11

LINK_FLAGS = -g -L ./ -L /usr/local/lib/ -L $(GUROBI_LIBDIR) -L ./flowstar-release/

OBJS_1 = ./src/sherlock.o ./src/network_computation.o ./src/gurobi_interface.o \
./src/configuration.o ./src/nodes.o ./src/computation_graph.o ./src/region_constraints.o \
./src/generate_constraints.o ./src/network_signatures.o  ./src/selective_binarization.o \
./src/parsing_onnx.o ./src/onnx.pb.o ./src/image_handler.o \
./src/neural_rule_analysis/AffineArithmeticExpression.o ./src/neural_rule_analysis/AffineArithmeticNoiseSymbols.o \
./src/neural_rule_analysis/Box.o ./src/neural_rule_analysis/Monomial.o \
./src/neural_rule_analysis/mpfiWrapper.o ./src/neural_rule_analysis/neuralRuleAnalysisInterfaceMain.o \
./src/neural_rule_analysis/Polynomial.o ./src/neural_rule_analysis/PolynomialApproximator.o \
./src/neural_rule_analysis/Tiling.o ./src/neural_rule_analysis/RangeToVariables.o \
./src/compute_flowpipes.o ./src/sherlock_poly.o ./src/polynomial_computations.o

DEPENDS = ${OBJECTS:.o=.d}


all: libs run_file
flocking : libs drone
dynamical_systems: libs ./ODE_reach_tubes/Ex_1
test_rule : libs ./ODE_reach_tubes/test_rule

libs: $(OBJS_1)
	ar rcs ./src/libsherlock.a $(OBJS_1)
	ranlib ./src/libsherlock.a
	cp ./src/*.h ./include


run_file: main.o $(OBJS_1)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

drone: \
	./systems_with_networks/flocking_controller/drone.o $(OBJS_1)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

./ODE_reach_tubes/Ex_1: ./ODE_reach_tubes/Ex_1.o $(OBJS_1)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

./ODE_reach_tubes/test_rule: ./src/neural_rule_analysis/main.o $(OBJS_1)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

%.o: %.cc
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.cpp
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.c
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f ./src/*.o ./src/neural_rule_analysis/*.o *.o ./run_file ./lib/* ./include/*.h \
	./ODE_reach_tubes/*.o

-include ${DEPENDS}
