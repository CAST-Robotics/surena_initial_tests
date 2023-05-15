#pragma once

#include"Eigen/Dense"
#include "Eigen/Core"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"
#include "yaml-cpp/yaml.h"
#include <ros/package.h>

using namespace Eigen;
using namespace std;

class MinJerk{
    public:
        MinJerk();
        virtual ~MinJerk();
        void setConfigPath(string config_path=ros::package::getPath("trajectory_planner") + "/config/config.yaml");
        void parseConfig(YAML::Node config);
        inline void setDt(double dt){dt_ = dt;}
        void setParams(double init_dsp, double dsp, double ssp, double final_dsp, double step_heigh);
        void setFootStepsData(vector<Vector3d> foot_steps, vector<double> foot_yaws);
        int getTrajSize();
        void cubicPolyTraj(const MatrixXd& way_points, const VectorXd& time_points, double dt, const MatrixXd& vel_points, MatrixXd& q);
        Vector4d genCubicCoeffs(const double pos_pts[], const double vel_pts[], double final_time);
        inline vector<double> getFootYaws(){return footYaws_;}
        inline double getFootStepCount(){return footStepCount_;}
        inline double getInitDSPDuration(){return initDSPDuration_;}
        inline double getFinalDSPDuration(){return finalDSPDuration_;}
        inline double getSSPDuration(){return SSPDuration_;}
        inline double getDSPDuration(){return DSPDuration_;}

    protected:
        static double dt_;
        YAML::Node config_;
        static double initDSPDuration_;
        static double DSPDuration_;
        static double SSPDuration_;
        static double finalDSPDuration_;
        static double stepHeight_;
        static vector<Vector3d> footSteps_;
        static vector<double> footYaws_;
        static int footStepCount_;
        static int trajSize_;
        static bool leftFirst_;
        static bool leftLast_;

        template <typename T>
        T* cubicInterpolate(T theta_ini, T theta_f, T theta_dot_ini, T theta_dot_f, double tf){
            /* 
                Returns Cubic Polynomial with the Given Boundary Conditions
                https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
            */
            T* coefs = new T[4]; // a0, a1, a2, a3
            coefs[0] = theta_ini;
            coefs[1] = theta_dot_ini;
            coefs[2] = 3/pow(tf,2) * (theta_f - theta_ini) - 1/tf * (2 * theta_dot_ini + theta_dot_f);
            coefs[3] = -2/pow(tf,3) * (theta_f - theta_ini) + 1/pow(tf,2) * (theta_dot_ini + theta_dot_f);
            return coefs;
        }

        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf);
        Vector3d* poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf, Vector3d x_d_ini, Vector3d _d_f, Vector3d x_dd_ini, Vector3d x_dd_f);
        Vector3d* ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf, double final_height);
        Vector3d* minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
        void write2File(Vector3d* input, int size, string file_name);
        void isLeftLast();
};