#ifndef sherlock_poly_h
#define sherlock_poly_h

#include "sherlock.h"
typedef map<uint32_t, double > _point_;

class polyhedral_abstraction
{
public:
  void optimize( region_constraints & input_polyhedron,
                 linear_inequality & optimal_direction,
                 computation_graph & neural_network,
                 _point_& contact_point, double & result);

  _point_ find_intersection_points( vector< linear_inequality > & lines);
  linear_inequality find_direction( vector < _point_ >& contact_points,
                                    _point_ & intersection_point);

  void tighten_polyhedron( region_constraints & input_polyhedron,
                           region_constraints & starting_polyhedron,
                           computation_graph & current_graph,
                           region_constraints & output_region);

  void split_polyhedron();
  void compute_polyhedrons( vector < region_constraints > & input_polyhedrons,
                            computation_graph & neural_network,
                            set < uint32_t > & current_output_neurons,
                            vector < region_constraints > & all_polyhedrons);

  void propagate_polyhedrons( computation_graph & neural_network,
                              region_constraints & input_polyhedron,
                              region_constraints & output_polyhedron,
                              set< uint32_t >& input_indices,
                              set< uint32_t >& output_indices);
};

void test_poly_abstr_simple(computation_graph & CG);

void drone_example(computation_graph & CG);

bool check_subset(set < uint32_t > & left_set,
                  set < uint32_t > & right_set);

bool check_subset( vector < uint32_t > & left_set,
                   set < uint32_t > & right_set);

void convert_vector_to_set( vector < uint32_t > & vector_in,
                            set < uint32_t > & set_out);


bool check_subset(
  set < uint32_t > & left_set,
  set < uint32_t > & right_set
);


bool check_subset(
  set < uint32_t > & left_set,
  vector < uint32_t > & right_set
);

#endif
