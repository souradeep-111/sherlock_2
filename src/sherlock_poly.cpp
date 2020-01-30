#include "sherlock_poly.h"

double improv_delta = 1e-1;
double trial_count = 1e2;
bool debug_sherlock_poly = true;

void polyhedral_abstraction :: optimize( region_constraints & input_polyhedron,
                                         linear_inequality & optimal_direction,
                                         computation_graph & neural_network,
                                         _point_& contact_point, double & result
)
{
  assert(!optimal_direction.empty());

  sherlock sherlock_handler(neural_network);
  sherlock_handler.maximize_in_direction(optimal_direction, input_polyhedron,
                        result, contact_point);

  sherlock_handler.clear();
}

_point_ polyhedral_abstraction :: find_intersection_points(
  vector< linear_inequality > & lines
)
{
  assert(! lines.empty());
  _point_ return_val;

  // Declare all the Gurobi variables
  GRBEnv * env_ptr = new GRBEnv();
  erase_line();
  env_ptr->set(GRB_IntParam_OutputFlag, 0);
  GRBModel * model_ptr = new GRBModel(*env_ptr);
  model_ptr->set(GRB_DoubleParam_IntFeasTol, sherlock_parameters.int_tolerance);

  map< uint32_t, GRBVar > gurobi_variables;
  vector< uint32_t > dimension_indices;
  lines[0].get_the_dimension_indices(dimension_indices);

  for(auto each_index : dimension_indices)
  {
    GRBVar gurobi_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY,
      0.0, GRB_CONTINUOUS, to_string(each_index));
    gurobi_variables[each_index] = gurobi_var;
  }

  for(auto & each_line : lines)
    each_line.add_equality_constraint_to_MILP_model(gurobi_variables, model_ptr);

  // Set Optimization direction to nothing
  GRBLinExpr objective_expr;
  objective_expr = 0;
  model_ptr->setObjective(objective_expr, GRB_MAXIMIZE);
  model_ptr->optimize();
  model_ptr->update();

  // Get a feasible solution
  if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
  {

      return_val.clear();
      for(auto & var : gurobi_variables)
        return_val[var.first] = var.second.get(GRB_DoubleAttr_X);
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
  {
      return_val.clear();
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD)
  {
    model_ptr->set(GRB_IntParam_DualReductions, 0);
    model_ptr->update();
    model_ptr->optimize();
    if( model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL )
    {
      return_val.clear();
      for(auto & var : gurobi_variables)
        return_val[var.first] = var.second.get(GRB_DoubleAttr_X);
    }
    else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
      return_val.clear();
    }
  }
  else
  {
      cout << "Some unkown Gurobi flag !" << endl;
      cout << "Flag returned - " << model_ptr->get(GRB_IntAttr_Status) << endl;
      assert(false);
  }


  // Delete the Gurobi stuff
  if(model_ptr)
    delete model_ptr;
  if(env_ptr)
    delete env_ptr;

  return return_val;
}

linear_inequality polyhedral_abstraction :: find_direction(
  vector < _point_ >& contact_points,
  _point_ & intersection_point
)
{
  // Build the Gurobi model and environment
  // Declare the variables involved in this thing
  // Set the constraints that contact points have to be on one side
  // Set the constraint that the intersection points have to be on another side
  // Get the solution, for the the linear inequality and return it

  assert(! contact_points.empty());
  assert(! intersection_point.empty());

  // Making sure that at least for the 1st contact point, the dimensions match
  for(auto each_pair : contact_points[0])
    assert(intersection_point.find(each_pair.first) != intersection_point.end());

  GRBEnv * env_ptr = new GRBEnv();
  erase_line();
  env_ptr->set(GRB_IntParam_OutputFlag, 0);
  GRBModel * model_ptr = new GRBModel(*env_ptr);
  model_ptr->set(GRB_DoubleParam_IntFeasTol, sherlock_parameters.int_tolerance);

  double data;
  map< uint32_t, GRBVar > gurobi_variables;

  // For the constant part of the inequality
  GRBVar gurobi_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY,
    0.0, GRB_CONTINUOUS, "c");
  gurobi_variables[-1] = gurobi_var;

  // For all the other dimensions involved
  for(auto each_pair : intersection_point)
  {
    GRBVar gurobi_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY,
      0.0, GRB_CONTINUOUS, to_string(each_pair.first));
    gurobi_variables[each_pair.first] = gurobi_var;
  }


  GRBLinExpr expr;
  // Constraint saying all the contact points have to be on one side
  for(auto each_point : contact_points)
  {
    // Basically the linear inequality has to be negative on the contact points
    expr = 0.0;

    for(auto value : each_point)
    {
      data = value.second;
      expr.addTerms(& data, & gurobi_variables[value.first], 1);
    }
    model_ptr->addConstr(expr, GRB_LESS_EQUAL, 0.0, "contact_point_constr");
  }
  // Constraint saying the intersection point has to be on the other side
  expr = 0.0;

  for(auto value : intersection_point)
  {
    data = value.second;
    expr.addTerms(& data, & gurobi_variables[value.first], 1);
  }
  model_ptr->addConstr(expr, GRB_GREATER_EQUAL, 0.0 + (improv_delta * 1e-2), "intersection_point_constr");


  model_ptr->setObjective(expr, GRB_MAXIMIZE);
  model_ptr->optimize();
  model_ptr->update();

  map < int, double > linear_exp;

  // Get a feasible solution
  if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
  {
      linear_exp.clear();
      for(auto & var : gurobi_variables)
        linear_exp[var.first] = var.second.get(GRB_DoubleAttr_X);
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
  {
      linear_exp.clear();
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD)
  {
    model_ptr->set(GRB_IntParam_DualReductions, 0);
    model_ptr->update();
    model_ptr->optimize();
    if( model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL )
    {
      linear_exp.clear();
      for(auto & var : gurobi_variables)
        linear_exp[var.first] = var.second.get(GRB_DoubleAttr_X);

    }
    else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
      linear_exp.clear();
    }
  }
  else
  {
      cout << "Some unkown Gurobi flag !" << endl;
      cout << "Flag returned - " << model_ptr->get(GRB_IntAttr_Status) << endl;
      assert(false);
  }


  // Delete the Gurobi stuff
  if(model_ptr)
    delete model_ptr;
  if(env_ptr)
    delete env_ptr;

  linear_inequality l(linear_exp);
  return l;

}

void polyhedral_abstraction :: tighten_polyhedron(
  region_constraints & input_polyhedron,
  region_constraints & starting_polyhedron,
  computation_graph & current_graph,
  region_constraints & output_region
)
{
  // Using the linear inequality data class, as a way to store equality stuff
  // as well.

  output_region.clear();
  output_region = starting_polyhedron;

  uint32_t index;

  linear_inequality optimal_direction;
  vector< linear_inequality > lines;

  _point_ new_contact_point, intersection_point;
  vector< _point_ > contact_points;

  double optimal_value, score;
  int space_dim = starting_polyhedron.get_space_dimension();

  index = 0;
  while(index < trial_count)
  {
    output_region.pick_random_directions(space_dim, lines, index);
    intersection_point = find_intersection_points(lines);
    if(output_region.check(intersection_point))
    {
      output_region.get_contact_points(contact_points);
      optimal_direction = find_direction(contact_points, intersection_point);

      if(!optimal_direction.empty())
      {
        optimize(input_polyhedron, optimal_direction, current_graph,
                new_contact_point, optimal_value);
        optimal_direction.negate();
        optimal_direction.update_bias(optimal_value);
        score = optimal_direction.evaluate(intersection_point);
        if(score > improv_delta)
        {
          output_region.add_direction_and_contact_point(optimal_direction, new_contact_point);
        }

      }
    }
    index++;
  }


}

void polyhedral_abstraction :: split_polyhedron()
{
  return ;
}

void polyhedral_abstraction :: compute_polyhedrons(
  vector < region_constraints > & input_polyhedrons,
  computation_graph & neural_network,
  set < uint32_t > & current_output_neurons,
  vector < region_constraints > & all_polyhedrons
)
{

  // To be removed when you have multiple ones
  assert(input_polyhedrons.size() == 1);

  region_constraints current_polyhedron, result_polyhedron, input_poly;
  input_poly = input_polyhedrons[0];

  // Building the region for axis parallel directions
  double max, min;
  map< uint32_t, pair< _point_, _point_> > contact_points;
  _point_ max_point, min_point, max_output, min_output;
  map< uint32_t , pair< double, double > > interval_limits;
  sherlock sherlock_instance(neural_network);



  for(auto each_output_neuron : current_output_neurons)
  {
    sherlock_instance.optimize_node_with_witness(
      each_output_neuron, true, input_poly, max, max_point);
    sherlock_instance.optimize_node_with_witness(
      each_output_neuron, false, input_poly, min, min_point);
    interval_limits[each_output_neuron] = make_pair(min, max);

    neural_network.evaluate_graph(min_point, min_output);
    neural_network.evaluate_graph(max_point, max_output);

    contact_points[each_output_neuron] = make_pair(min_output, max_output);
  }

  current_polyhedron.clear();
  current_polyhedron.create_region_from_interval(interval_limits, contact_points);

  tighten_polyhedron(input_poly, current_polyhedron, neural_network, result_polyhedron);

  all_polyhedrons.clear();
  all_polyhedrons.push_back(result_polyhedron);

}

void polyhedral_abstraction :: propagate_polyhedrons(
  computation_graph & neural_network,
  region_constraints & input_polyhedron,
  region_constraints & output_polyhedron,
  set< uint32_t >& input_indices,
  set< uint32_t >& output_indices
)
{

  set < uint32_t > current_input_neurons, current_output_neurons, sub_graph_inputs;
  vector< uint32_t > input_neurons, output_neurons;

  current_input_neurons = input_indices;
  current_output_neurons = input_indices;

  vector < region_constraints > current_input_polyhedrons, current_output_polyhedrons;
  computation_graph sub_graph;

  current_input_polyhedrons.push_back(input_polyhedron);

  while(!check_subset(output_indices, current_output_neurons) )
  {
    neural_network.return_next_layer_from_set(current_input_neurons,
                    current_output_neurons);

    // So basically there is nothing interesting left in the network
    // we just burn our way through the rest of it
    if(current_output_neurons.empty())
      current_output_neurons = output_indices;

    neural_network.extract_graph(current_output_neurons, current_input_neurons, sub_graph);

    compute_polyhedrons(current_input_polyhedrons, sub_graph, current_output_neurons,
                        current_output_polyhedrons);

    current_input_polyhedrons = current_output_polyhedrons;
    current_input_neurons = current_output_neurons;
  }

  // For single polyhedron case
  output_polyhedron = current_output_polyhedrons[0];
  return;
}

bool check_subset(
  set < uint32_t > & left_set,
  set < uint32_t > & right_set
)
{
  assert(!right_set.empty());


  for(auto each_number : left_set)
  {
    if(right_set.find(each_number) == right_set.end())
      return false;
  }

  return true;
}

bool check_subset(
  vector < uint32_t > & left_set,
  set < uint32_t > & right_set
)
{
  assert(!right_set.empty());
  assert(left_set.size() <= right_set.size());
  for(auto each_number : left_set)
  {
    if(right_set.find(each_number) == right_set.end())
      return false;
  }
  return true;
}

void convert_vector_to_set(
  vector < uint32_t > & vector_in,
  set < uint32_t > & set_out
)
{
  set_out.clear();

  for(auto each_element : vector_in)
    set_out.insert(each_element);

}


void test_poly_abstr_simple(computation_graph & CG)
{
  map<uint32_t , double > inputs, outputs;
  inputs[1] = 1.0;
  inputs[2] = 6.0;
  // Output index = 38;

  set < uint32_t > input_indices, output_indices;
  input_indices.insert(1);
  input_indices.insert(2);
  output_indices.insert(38);


  CG.evaluate_graph(inputs, outputs);
  cout << "Output - " << outputs[38] << endl;

  map< uint32_t, pair< double, double > > input_interval;
  input_interval[1] = make_pair(0,1);
  input_interval[2] = make_pair(0,1);
  region_constraints input_polyhedron, output_polyhedron;
  input_polyhedron.create_region_from_interval(input_interval);

  polyhedral_abstraction sherlock_poly;
  sherlock_poly.propagate_polyhedrons(CG, input_polyhedron, output_polyhedron,
    input_indices, output_indices);

  cout << " ------ Output polyhedron computed ----- " << endl;
  output_polyhedron.print();
}

void drone_example(computation_graph & CG)
{
  map<uint32_t , double > inputs, outputs;
  inputs[1] = 0.0;
  inputs[2] = -3.2342;
  inputs[3] = 0.0;
  inputs[4] = -0.1815;
  inputs[5] = -0.2004;
  inputs[6] = 0.6001;
  inputs[7] = -0.4803;
  inputs[8] = -0.1372;

  CG.evaluate_graph(inputs, outputs);
  // Expected: 0.6991
  cout << "Output - " << outputs[1009] << endl;
  // Expected: -0.0676
  cout << "Output - " << outputs[1010] << endl;


  inputs[1] = 0.0;
  inputs[2] = 2.3855;
  inputs[3] = 0.0;
  inputs[4] = -2.5104;
  inputs[5] = -0.7279;
  inputs[6] = 0.1594;
  inputs[7] = 0.7386;
  inputs[8] = 0.0997;

  CG.evaluate_graph(inputs, outputs);
  // Expected : 0.9333
  cout << "Output - " << outputs[1009] << endl;
  // Expected : -0.8953
  cout << "Output - " << outputs[1010] << endl;


  inputs[1] = 0.0;
  inputs[2] = -0.3683;
  inputs[3] = 0.0;
  inputs[4] = 3.5940;
  inputs[5] = 0.0265;
  inputs[6] = -0.8481;
  inputs[7] = -0.1964;
  inputs[8] = -0.5202;

  CG.evaluate_graph(inputs, outputs);
  // Expected : -0.3540
  cout << "Output - " << outputs[1009] << endl;
  // Expected : 0.9458
  cout << "Output - " << outputs[1010] << endl;


  inputs[1] = 0.0;
  inputs[2] = -3.9443;
  inputs[3] = 0.0;
  inputs[4] = 2.0550;
  inputs[5] = 0.8896;
  inputs[6] = -0.0215;
  inputs[7] = -0.0183;
  inputs[8] = -0.3246;

  CG.evaluate_graph(inputs, outputs);
  // Expected: -1.3783
  cout << "Output - " << outputs[1009] << endl;
  // Expected: 0.5207
  cout << "Output - " << outputs[1010] << endl;


  inputs[1] = 0.0;
  inputs[2] = 4.1208;
  inputs[3] = 0.0;
  inputs[4] = -1.8342;
  inputs[5] = -0.2205;
  inputs[6] = -0.1922;
  inputs[7] = -0.5166;
  inputs[8] = -0.8071;

  CG.evaluate_graph(inputs, outputs);
  // Expected: 1.1930
  cout << "Output - " << outputs[1009] << endl;
  // Expected: -0.6009
  cout << "Output - " << outputs[1010] << endl;

}
