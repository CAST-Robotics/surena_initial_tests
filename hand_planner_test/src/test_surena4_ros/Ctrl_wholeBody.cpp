#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp" 
#include "../Eigen/Core" 
#include "../Eigen/Cholesky" 
#include "../Eigen/LU" 
#include <math.h>
#include <iostream>
#include <vector>
#include "fstream"
#include <string>

using namespace Eigen;
using namespace std;

// QP parameters
MatrixXd G;
VectorXd g;
MatrixXd CI;
VectorXd ci0;
MatrixXd CE;
VectorXd ce0;
VectorXd qpresult; 

void doQP_wholeBody(double t, double T, double omega, double L_min, double L_max, double W_min, double W_max, double T_min, double T_max, MatrixXd b_nom, MatrixXd Xi_meas, MatrixXd r_vrp, MatrixXd Xi_err){
    // cost function
    double Wux, Wuy = 1;
    double Wbx, Wby = 5;
    double WT = .05;
    G.resize(5,5);
    G << 2*Wux, 0, 0, 0, 0,
         0, 2*Wbx, 0, 0, 0,
         0, 0, 2*Wuy, 0, 0,
         0, 0, 0, 2*Wby, 0,
         0, 0, 0, 0,  2*WT;
    g.resize(5,1);
    // g = MatrixXd();

    // Inequality constraint
    CI.resize(6,5);
    CI<<1, 0,  0, 0,  0,
       -1, 0,  0, 0,  0,
        0, 0,  1, 0,  0,
        0, 0, -1, 0,  0,
        0, 0,  0, 0,  1,
        0, 0,  0, 0, -1; 
    ci0.resize(6);
    ci0 <<  L_max - r_vrp(0),
          -(L_min - r_vrp(0)),
            W_max - r_vrp(0),
          -(W_min - r_vrp(0)),
            exp(omega*T_max) - exp(omega*T),
          -(exp(omega*T_min) - exp(omega*T));

    // // Equality constraint
    CE.resize(2,5);
    CE << 1, 1, 0, 0, -Xi_meas(0)*exp(-omega*t),
          0, 0, 1, 1, -Xi_meas(1)*exp(-omega*t);
           

    ce0.resize(2,1);
    ce0 << Xi_err(0)*exp(omega*(T-t)),
           Xi_err(1)*exp(omega*(T-t));
    
    // Solving OPT
    qpresult.resize(5);
    double optCosts;
    optCosts = solve_quadprog(G, g, CE.transpose(), ce0, CI.transpose(), ci0, qpresult);
    cout << optCosts<<endl;
}; 









int main(){
    MatrixXd b_nom(1,2);
    b_nom << 0.0372, 0.0186;
    MatrixXd Xi_meas(1,2);
    Xi_meas << 0.0372, 0.0186;
    MatrixXd r_vrp(1,2);
    r_vrp << 0.325, -0.1;
    MatrixXd Xi_err(1,2);
    Xi_err << 0, 0;
    //(double t, double T, double omega, double L_min, double L_max, double W_min, double W_max, double T_min, double T_max, MatrixXd b_nom, MatrixXd Xi_meas, MatrixXd r_vrp, MatrixXd Xi_err)
    doQP_wholeBody(0.001, 0.65, 3.5, -0.5, 0.5, -0.3, 0, 0.3, 1, b_nom, Xi_meas, r_vrp, Xi_err);

    return 0;
}