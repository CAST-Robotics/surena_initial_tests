#ifndef HANDWRITING_H
#define HANDWRITING_H
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <map>
using namespace std;

class handWriting
{
public:
    double a;
    double a_x;
    double a_y;
    double a_y_n;
    double a_z;
    double b;
    double c;
    double d;
    double P_start;
    double P_end;
    double dt;
    double t;
    double T;
    double L;
    double V;
    double P;
    double V_x;
    double V_y;
    double V_z;
    map<string,double>Z_start;
    map<string,double>Y_start;
    map<string,double>Z_end;
    map<string,double>Y_end;
    handWriting();
    double saturate(double a, double min, double max);
    double move2pose(double max, double t_local, double T_start, double T_end);
    double move_to_diff(double max, double t_local, double T_start, double T_end);
    double Velocity(double _L, double _t, double _T);
    double Position(double _L, double _t, double _T);
    void move2next(string current, string next);
    double move2zero(double theta,double t,double T_home);
    void defineLetter(string letter, double z_start, double y_start, double z_end, double y_end);

    void Write_I();
    void Write_A();
    void Write_M();
    void Write_S();
    void Write_U();
    void Write_R();
    void Write_E();
    void Write_N();
    void Write_H();
    void Write_V();
    void Write_X();
    void Write_Z();
    void Write_L();
    void Write_T();
    void Write_Y();
    void Write_F();
    void Write_J();
    void Write_B();
    void Write_W();
    void Write_P();
    void Write_D();
    void Write_K();
    void Write_x_axis();
    void Write_y_axis(double a_y ,double T);
    void Write_x_axis_n();
    void Write_y_axis_n(double a_y_n ,double T);


    void moveback();
};

#endif // HANDWRITING_H
