#include<boost/python.hpp> 
#include "solver.h" 

Vars vars; 
Params params; 
Workspace work; 
Settings settings; 

int flags = 0; 

boost::python::list runController(      // these are all the paramters that get sent to the optimizer 
                  boost::python::list A, 
				  boost::python::list B, 
				  boost::python::list Q1, 
				  boost::python::list Q2, 
				  boost::python::list Q3, 
				  boost::python::list R1, 
				  boost::python::list R2, 
				  boost::python::list q_0, 
				  boost::python::list q_goal, 
				  boost::python::list tau_prev)
{ 
  if (flags == 0) 
    { 
      set_defaults(); 
      setup_indexing(); 
      flags = 1; 
      settings.verbose = 0; 
      settings.eps = 1e-4; // this could be changed to e-6 to be more accurate if necessary 
    } 

  // define the optimization parameters. 

  int i; 

  for (i = 0; i < boost::python::len(A); i++) 
    { 
      params.A[i] = boost::python::extract<double>(A[i]); 
    } 

  
  for (i = 0; i < boost::python::len(B); i++) 
    { 
      params.B[i] = boost::python::extract<double>(B[i]); 
    } 


  for (i = 0; i < boost::python::len(Q1); i++) 
    { 
      params.Q1[i] = boost::python::extract<double>(Q1[i]); 
    } 

  for (i = 0; i < boost::python::len(Q2); i++) 
    { 
      params.Q2[i] = boost::python::extract<double>(Q2[i]); 
    } 

  for (i = 0; i < boost::python::len(Q3); i++) 
    { 
      params.Q3[i] = boost::python::extract<double>(Q3[i]); 
    } 

  for (i = 0; i < boost::python::len(q_0); i++) 
    { 
      params.q_0[i] = boost::python::extract<double>(q_0[i]); 
    } 


  for (i = 0; i < boost::python::len(q_goal); i++) 
    { 
      params.q_goal[i] = boost::python::extract<double>(q_goal[i]); 
    } 

  for (i = 0; i < boost::python::len(tau_prev); i++) 
    { 
      params.tau_prev[i] = boost::python::extract<double>(tau_prev[i]); 
    } 

  for (i = 0; i < boost::python::len(R1); i++) 
    { 
      params.R1[i] = boost::python::extract<double>(R1[i]); 
    } 

  for (i = 0; i < boost::python::len(R2); i++) 
    { 
      params.R2[i] = boost::python::extract<double>(R2[i]); 
    } 

  solve(); 


  // This is the stuff that you want returned from the solver 

  boost::python::list all_data; 

  boost::python::list tau_1_cmd; 
  boost::python::list tau_2_cmd; 

  if (work.converged == 1) 
  { 
      tau_1_cmd.append(vars.u[0][0]);
      tau_2_cmd.append(vars.u[0][1]);
  } 
  else 
  { 
    tau_1_cmd.append(0.0); 
    tau_2_cmd.append(0.0); 
  } 

  all_data.append(tau_1_cmd); 
  all_data.append(tau_2_cmd); 

  return all_data; 
} 

char const* greet() 
{ 
  printf("got into greet"); 
  return "hello, world (and you)"; 
} 

// this is what tells boost what the function names and definitions should be (DON'T DELETE) 
BOOST_PYTHON_MODULE(double_pendulum_mpc_solver) 
{ 
  using namespace boost::python; 
  def("greet", greet); 
  def("runController", runController); 
} 
