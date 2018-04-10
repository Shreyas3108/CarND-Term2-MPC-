#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// TODO: Set the timestep length and duration
size_t N = 10; // shouldn't be more than 15 as discussed in the video 
double dt = 0.1; // higher value would make this more unstabel and lower value would result in lesser delay and higher computation. My intuition on this part was similar to learning rates we use in ML 

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67; //dervied value from deg2rad(25) 

const double ref_cte = 0; // initial cross track error which we want as 0 

const double ref_epsi = 0; // initial error which we again would want as 0 

const double ref_v = 100; // Velocity at which the car would drive. 


// initialization of variables 
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N; //cross track error 
const size_t epsi_start = cte_start + N; // error
const size_t delta_start = epsi_start + N; // angle 
const size_t a_start = delta_start + N - 1; // acc 


// This part has been taken from the lessons. Just tuned the weights rest all are more or less the same. 
class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	
	  fg[0] = 0;
     
	 // cost parameters taken on the basis of state of the car. 

    for(unsigned int i = 0; i < N; i++ ) {

      fg[0] += 1100*CppAD::pow(vars[cte_start + i] - ref_cte, 2); // 2000 initial as discussed in the video , iterated to 1700 , 1500 (where car worked fine on road but had jerks. Hence settled for 1100. 
      fg[0] += 1100*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); // 2000 as discussed in the video , similar explanation as above 
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);

    }

    // Minimize actuators which would enable the car to move straight when required and not let the car move left and right always 
    for (unsigned int i = 0; i< N - 1; i++) {
      fg[0] += 55*CppAD::pow(vars[delta_start + i], 2); // inital 200  , 100 then 50 
      fg[0] += 55*CppAD::pow(vars[a_start + i], 2); // 100 , 10 , 50 
    }

    // There can be a gap in actuators which could lead to the car running over the road/fence. This loop below reduces the gap between the actuators 

    for (unsigned int i = 0; i < N - 2; i++) {

      fg[0] += 250000*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2); // inital 60k , 100k , 250k 
      fg[0] += 5000*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2); // 20k , 10k 5k 

    }



    // Initial parameter constraint 

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (unsigned int t = 1; t < N; t++) {

      // The state at time t+1 .

      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];


      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);
    }
  }
  
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6*N + (N-1)*2 ;
  // TODO: Set the number of constraints
  size_t n_constraints = N*6 ;
  
  // initializing the variables from the states as stated in the main function to solve. 
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];
  
  

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  //using unsigned int to avoid warnings 
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  
	// TODO: Set lower and upper limits for variables.
	//using the maximum value as uppare and lower limit. 
  for (unsigned int i = 0; i < delta_start; i++ ) 
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 to 25
  // Setting the bound's for steering. Hence angles. 

  for (unsigned int i = delta_start; i < a_start; i++ ) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.43632*Lf;
  }


  for (unsigned int i = a_start; i < n_vars; i++ ) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) 
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }



  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y ; 
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;



  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> solves;
  solves.push_back(solution.x[delta_start]);
  solves.push_back(solution.x[a_start]);
  for (unsigned int i = 0; i < N - 2; i++ ) {
    solves.push_back(solution.x[x_start + i + 1]);
    solves.push_back(solution.x[y_start + i + 1]);
  }
  return solves;
}