#ifndef sherlock_h
#define sherlock_h

#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <queue>
#include <math.h>
#include <string>
#include "computation_graph.h"
#include "generate_constraints.h"
#include "configuration.h"
#include "region_constraints.h"
#include "network_computation.h"
#include "selective_binarization.h"
#include "parsing_onnx.h"
#include "image_handler.h"

using namespace std;

class sherlock
{
private:
  computation_graph neural_network;
  constraints_stack network_constraints;
public:
  uint32_t nodes_explored;
  sherlock();
  sherlock(computation_graph & CG);
  void clear();
  void set_computation_graph(computation_graph & CG);
  void optimize_node(uint32_t node_index, bool direction,
                      region_constraints & input_region,
                      double & optima_achieved);

  void optimize_constrained(uint32_t node_index, bool direction,
                                 region_constraints & input_region,
                                 vector< linear_inequality > & inequalities,
                                 double & optima_achieved);

  void gradient_driven_optimization(uint32_t node_index,
                                    region_constraints & input_region,
                                    bool direction, double & optima);
  void compute_output_range(uint32_t node_index,
                            region_constraints & input_region,
                            pair < double, double >& output_range);
  void compute_output_region(region_constraints & input_region,
                             region_constraints & output_region);

  void perform_gradient_search(uint32_t node_index, bool direction,
                                region_constraints & region,
                                map< uint32_t, double > & starting_point, double & val);

  void perform_gradient_search_with_random_restarts(uint32_t node_index,
                               bool direction, region_constraints & region,
                               map< uint32_t, double > & starting_point, double & val);

  void compute_output_range_by_sampling(region_constraints & input_region,
                                        uint32_t output_node_index,
                                        pair < double , double > & output_range,
                                        uint32_t sample_count);

  void increment_point_in_direction(map<uint32_t, double >& current_values,
                                        map<uint32_t, double > direction,
                                        region_constraints& region);

  void increment_point_in_direction(map<uint32_t, double >& current_values,
                                    map<uint32_t, double > direction);

  bool increment_point_in_direction(map<uint32_t, double >& current_values, double step_size,
                                    map<uint32_t, double > direction, region_constraints& region);

  bool return_best_effort_random_counter_example(bool direction,
                                    map< uint32_t , double >& current_point,
                                    double& val_curr, uint32_t node_index,
                                    region_constraints & region);

  void add_constraint(linear_inequality & lin_ineq);

  bool prove_bounds(uint32_t node_index, double bound, bool direction,
                    region_constraints& input_region, set< uint32_t >& binarized_neurons);

};


void create_computation_graph_from_file(string filename,
                                        computation_graph & CG,
                                        bool has_output_relu,
                                        vector<uint32_t>& input_node_indices,
                                        vector<uint32_t>& output_node_indices);

void test_network_1(computation_graph & CG);
void test_network_2(computation_graph & CG);
void test_network_sigmoid(computation_graph & CG);
void test_network_tanh(computation_graph & CG);
#endif
