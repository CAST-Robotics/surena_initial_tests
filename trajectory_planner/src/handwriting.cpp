#include "handwriting.h"


handWriting::handWriting()
{
    a=.02;
    a_x=0.1;
    // a_y=0.3;
    // a_y_n=0.25;
    a_z=0.15;
    b=2*a;
    c=a/2;
    d=.03;
    dt=.005;

    defineLetter("I",b,a/2,0,3*a/4);
    defineLetter("A",0,0,b/2,3*a/4);
    defineLetter("M",0,0,0,a);
    defineLetter("S",.75*b,a,.25*b,0);
    defineLetter("U",b,0,b,a);
    defineLetter("R",b,0,0,a);
    defineLetter("E",b,a,b/2,a);
    defineLetter("N",0,0,b,a);
    defineLetter("H",b,0,b/2,a);
    defineLetter("V",b,0,b,a);
    defineLetter("X",b,a,0,a);
    defineLetter("Z",b,0,0,a);
    defineLetter("L",b,0,0,a);
    defineLetter("Y",b,0,0,a/2);
    defineLetter("T",b,a/2,b,a);
    defineLetter("F",b,a,0,3*a/4);//????????
    defineLetter("J",b,a/4,b/4,0);
    defineLetter("B",b,0,0,0);
    defineLetter("W",3*b/4,0,3*b/4,a);
    defineLetter("P",b,0,b/2,0);
    defineLetter("D",b,0,0,0);
    defineLetter("K",b,0,0,a);
    defineLetter("x_axis",0,0,0,a);
    defineLetter("y_axis",0,0,0,a_y);
    defineLetter("x_axis_n",0,0,0,0);
    defineLetter("y_axis_n",0,a_y_n,0,0);


}

double handWriting::saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
}

double handWriting::move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
}

double handWriting::move_to_diff(double max,double t_local,double T_start ,double T_end){
    double c3=(10*max)/pow(T_end-T_start,3);
    double c4=-(15*max)/pow(T_end-T_start,4);
    double c5=(6*max)/pow(T_end-T_start,5);
    double thetad=0;
    if(t_local<T_start){thetad=0;}
    else if (t_local<T_end){thetad=c3*pow(t_local-T_start,2)*3+c4*pow(t_local-T_start,3)*4+c5*pow(t_local-T_start,4)*5;}
    else{thetad=0;}
    return thetad;
}

double handWriting::Velocity(double _L,double _t, double _T){

    double _V=30*_L/pow(_T,3)*pow(_t,2)-60*_L/pow(_T,4)*pow(_t,3)+30*_L/pow(_T,5)*pow(_t,4);
    return _V;
}

double handWriting::Position(double _L,double _t, double _T){
    double _P=10*_L/pow(_T,3)*pow(_t,3)-15*_L/pow(_T,4)*pow(_t,4)+6*_L/pow(_T,5)*pow(_t,5);
    return _P;
}

void handWriting::defineLetter(string letter,double z_start,double y_start,double z_end,double y_end){
    Z_start.insert({letter,z_start});
    Y_start.insert({letter,y_start});
    Z_end.insert({letter,z_end});
    Y_end.insert({letter,y_end});
}


void handWriting::Write_I(){

    double T1=T/5;
    double T2=T/5;
    double T3=T/5;
    double T4=T/5;
    double T5=T/5;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }

    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(-a/4,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }

    else if(t<T1+T2+T3){
        L=a/2;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    else if(t<T1+T2+T3+T4){
        V_x=move_to_diff(-d,t,T1+T2+T3,T1+T2+T3+T4/2)-move_to_diff(-d,t,T1+T2+T3+T4/2,T1+T2+T3+T4);
        V_y=move_to_diff(-a/2,t,T1+T2+T3,T1+T2+T3+T4);
        V_z=move_to_diff(-b,t,T1+T2+T3,T1+T2+T3+T4);
    }

    else if(t<T1+T2+T3+T4+T5){
        L=a/2;
        V=Velocity(L,t-T1-T2-T3-T4,T5);
        P=Position(L,t-T1-T2-T3-T4,T5);
        V_x=0;
        V_y=V;
        V_z=0;
    }   
    t+=dt;
}


void handWriting::Write_A(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;

    if(t<T1){
        L=sqrt(a*a/4+b*b);
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/2;
        V_z=V*b/L;
    }
    else if(t<T1+T2){
        L=sqrt(a*a/4+b*b);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L/2;
        V_z=-V*b/L;
    }
    else if(t<T1+T2+T3){

        V_x=move_to_diff(-d,t,T1+T2,T1+T2+T3/2)-move_to_diff(-d,t,T1+T2+T3/2,T1+T2+T3);
        V_y=move_to_diff(-3*a/4,t,T1+T2,T1+T2+T3);
        V_z=move_to_diff(b/2,t,T1+T2,T1+T2+T3);
    }

    else if(t<T1+T2+T3+T4){
        L=a/2;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);

        V_x=0;
        V_y=V;
        V_z=0;
    }
    t+=dt;
}


void handWriting::Write_M(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;


    if(t<T1){
        L=sqrt(a*a/16+b*b);
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/4;
        V_z=V*b/L;
    }
    else if(t<T1+T2){
        L=sqrt(a*a/16+b*b);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L/4;
        V_z=-V*b/L;

    }
    else if(t<T1+T2+T3){

        L=sqrt(a*a/16+b*b);
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V*a/L/4;
        V_z=V*b/L;
    }

    else if(t<T1+T2+T3+T4){
        L=sqrt(a*a/16+b*b);
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V*a/L/4;
        V_z=-V*b/L;
    }
    t+=dt;
}

void handWriting::Write_S(){

    L=1.5*M_PI*(a+b/2)/2;


    V=Velocity(L,t,T);
    P=Position(L,t,T);
    if(t<T/2){
        V_x=0;
        V_y=-V*sin(3*P/L*M_PI);
        V_z=V*cos(3*P/L*M_PI);
    }
    else{
        V_x=0;
        V_y=V*cos(-3*(P-L/2)/L*M_PI);
        V_z=V*sin(-3*(P-L/2)/L*M_PI);
    }
    t+=dt;
}

void handWriting::Write_U(){

    L=.5*M_PI*(a+b)/2+b+b/8;

    V=Velocity(L,t,T);
    P=Position(L,t,T);

    if(P<b/2+b/16){
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(P<b/2+b/16+.5*M_PI*(a+b)/2){

        V_x=0;
        V_y=a/((a+b)/4)*sin((P-b/16-b/2)/((a+b)/4));
        V_z=-b/((a+b)/4)*cos((P-b/16-b/2)/((a+b)/4));

        double M=sqrt(V_x*V_x+V_y*V_y+V_z*V_z);
        V_y=V_y*V/M;
        V_z=V_z*V/M;

    }
    else{
        V_x=0;
        V_y=0;
        V_z=V;
    }
    t+=dt;
}

void handWriting::Write_R(){

    double T1=T/4+T/16;
    double T2=T/4;
    double T3=T/4+T/16;
    double T4=T/8;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){
        L=b;
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=0;
        V_z=V;
    }
    else if(t<T1+T2+T3){
        L=(a+b)/4*M_PI;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=b/((a+b)/4)*cos((P)/((a+b)/4));
        V_z=-a/((a+b)/4)*sin((P)/((a+b)/4));


        double M=sqrt(V_x*V_x+V_y*V_y+V_z*V_z);
        V_y=V_y*V/M;
        V_z=V_z*V/M;

    }
    else if(t<T1+T2+T3+T4){
        L=a*sqrt(2)-a/8*sqrt(2);
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V/sqrt(2);
        V_z=-V/sqrt(2);
    }

    t+=dt;
}

void handWriting::Write_E(){

    double T1=T/5;
    double T2=T/5;
    double T3=T/5;
    double T4=T/5;
    double T5=T/5;


    if(t<T1){
        L=a;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=-V;
        V_z=0;
    }
    else if(t<T1+T2){
        L=b;
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=0;
        V_z=-V;

    }
    else if(t<T1+T2+T3){
        L=a;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }
    else if(t<T1+T2+T3+T4){
        L=a*sqrt(2);
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=move_to_diff(-d,t,T1+T2+T3,T1+T2+T3+T4/2)-move_to_diff(-d,t,T1+T2+T3+T4/2,T1+T2+T3+T4);
        V_y=-V/sqrt(2);
        V_z=V/sqrt(2);
    }

    else if(t<T1+T2+T3+T4+T5){
        L=a;
        V=Velocity(L,t-T1-T2-T3-T4,T5);
        P=Position(L,t-T1-T2-T3-T4,T5);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    t+=dt;
}

void handWriting::Write_N(){

    double T1=T/3;
    double T2=T/3;
    double T3=T/3;



    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=V;
    }
    else if(t<T1+T2){
        L=sqrt(a*a+b*b);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L;
        V_z=-V*b/L;

    }
    else if(t<T1+T2+T3){
        L=b;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=0;
        V_z=V;
    }

    t+=dt;
}

void handWriting::Write_H(){

    double T1=T/5;
    double T2=T/5;
    double T3=T/5;
    double T4=T/5;
    double T5=T/5;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(a,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }
    else if(t<T1+T2+T3){

        L=b;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=0;
        V_z=-V;
    }

    else if(t<T1+T2+T3+T4){
        V_x=move_to_diff(-d,t,T1+T2+T3,T1+T2+T3+T4/2)-move_to_diff(-d,t,T1+T2+T3+T4/2,T1+T2+T3+T4);
        V_y=move_to_diff(-a,t,T1+T2+T3,T1+T2+T3+T4);
        V_z=move_to_diff(b/2,t,T1+T2+T3,T1+T2+T3+T4);
    }

    else if(t<T1+T2+T3+T4+T5){

        L=a;
        V=Velocity(L,t-T1-T2-T3-T4,T5);
        P=Position(L,t-T1-T2-T3-T4,T5);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    t+=dt;
}

void handWriting::Write_V(){

    double T1=T/2;
    double T2=T/2;



    if(t<T1){
        L=sqrt(a*a/4+b*b);
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/2;
        V_z=-V*b/L;
    }
    else if(t<T1+T2){
        L=sqrt(a*a/4+b*b);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L/2;
        V_z=V*b/L;

    }

    t+=dt;
}

void handWriting::Write_X(){

    double T1=T/3;
    double T2=T/3;
    double T3=T/3;



    if(t<T1){
        L=sqrt(a*a+b*b);
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=-V*a/L;
        V_z=-V*b/L;
    }
    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(0,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }
    else if(t<T1+T2+T3){
        L=sqrt(a*a+b*b);
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V*a/L;
        V_z=-V*b/L;

    }

    t+=dt;
}

void handWriting::Write_Z(){

    double T1=T/3;
    double T2=T/3;
    double T3=T/3;

    if(t<T1){
        L=a;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V;
        V_z=0;
    }
    else if(t<T1+T2){
        L=sqrt(a*a+b*b);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=-V*a/L;
        V_z=-V*b/L;

    }
    else if(t<T1+T2+T3){
        L=a;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }
}

void handWriting::Write_Y(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;

    if(t<T1){
        L=sqrt((a*a/4)+(b*b/4));
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/2;
        V_z=-V*b/L/2;
    }
    else if(t<T1+T2){
        L=sqrt((a*a/4)+(b*b/4));
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L/2;
        V_z=V*b/L/2;
    }

    else if(t<T1+T2+T3){
        V_x=move_to_diff(-d,t,T1+T2,T1+T2+T3/2)-move_to_diff(-d,t,T1+T2+T3/2,T1+T2+T3);
        V_y=move_to_diff(-a/2,t,T1+T2,T1+T2+T3);
        V_z=move_to_diff(-b/2,t,T1+T2,T1+T2+T3);
    }

    else if(t<T1+T2+T3+T4){
        L=b/2;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=0;
        V_z=-V;
    }

    t+=dt;
}

void handWriting::Write_T(){

    double T1=T/3;
    double T2=T/3;
    double T3=T/3;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(-a/2,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }
    else if(t<T1+T2+T3){

        L=a;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }
    t+=dt;

}


void handWriting::Write_F(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;


    if(t<T1){
        L=a;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=-V;
        V_z=0;
    }
    else if(t<T1+T2){
        L=b;
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=0;
        V_z=-V;

    }
    else if(t<T1+T2+T3){

        V_x=move_to_diff(-d,t,T1+T2,T1+T2+T3/2)-move_to_diff(-d,t,T1+T2+T3/2,T1+T2+T3);
        V_y=move_to_diff(0,t,T1+T2,T1+T2+T3);
        V_z=move_to_diff(b/2,t,T1+T2,T1+T2+T3);
    }

    else if(t<T1+T2+T3+T4){
        L=3*a/4;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);

        V_x=0;
        V_y=V;
        V_z=0;

    }
    t+=dt;
}

void handWriting::Write_J(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;

    if(t<T1){
        L=a/2;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(-a/4,t,T1,T1+T2);
        V_z=move_to_diff(0,t,T1,T1+T2);
    }

    else if(t<T1+T2+T3){
        L=3*b/4;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=0;
        V_z=-V;
    }

    else if(t<T1+T2+T3+T4){
        L=M_PI*b/4;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=-V*sin(P/L*M_PI);
        V_z=-V*cos(P/L*M_PI);
    }

    t+=dt;
}

void handWriting::Write_L(){

    double T1=T/2;
    double T2=T/2;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){
        L=a;
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V;
        V_z=0;

    }
    
    t+=dt;
}

void handWriting::Write_B(){

    double T1=T/8;
    double T2=T/8;
    double T3=T/8;
    double T4=T/8;
    double T5=T/8;
    double T6=T/8;
    double T7=T/8;
    double T8=T/8;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }

    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(0,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }

    else if(t<T1+T2+T3){
        L=3*b/16;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    else if(t<T1+T2+T3+T4){
        L=M_PI*3*b/16;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V*cos(P/L*M_PI);
        V_z=-V*sin(P/L*M_PI);
    }

    else if(t<T1+T2+T3+T4+T5){
        L=3*b/16;
        V=Velocity(L,t-T1-T2-T3-T4,T5);
        P=Position(L,t-T1-T2-T3-T4,T5);
        V_x=0;
        V_y=-V;
        V_z=0;
    }

    else if(t<T1+T2+T3+T4+T+T6){
        L=3*b/16;
        V=Velocity(L,t-T1-T2-T3-T4-T5,T6);
        P=Position(L,t-T1-T2-T3-T4-T5,T6);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    else if(t<T1+T2+T3+T4+T+T6+T7){
        L=M_PI*5*b/16;
        V=Velocity(L,t-T1-T2-T3-T4-T5-T6,T7);
        P=Position(L,t-T1-T2-T3-T4-T5-T6,T7);
        V_x=0;
        V_y=V*cos(P/L*M_PI);
        V_z=-V*sin(P/L*M_PI);
    }
    else if(t<T1+T2+T3+T4+T+T6+T7+T8){
        L=3*b/16;
        V=Velocity(L,t-T1-T2-T3-T4-T5-T6-T7,T8);
        P=Position(L,t-T1-T2-T3-T4-T5-T6-T7,T8);
        V_x=0;
        V_y=-V;
        V_z=0;
    }

    t+=dt;
}

void handWriting::Write_W(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;


    if(t<T1){
        L=sqrt(a*a/16+9*b*b/16);
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/4;
        V_z=-V*3*b/L/4;
    }
    else if(t<T1+T2){
        L=sqrt(a*a/16+b*b/4);
        V=Velocity(L,t-T1,T2);
        P=Position(L,t-T1,T2);
        V_x=0;
        V_y=V*a/L/4;
        V_z=V*b/L/2;

    }
    else if(t<T1+T2+T3){

        L=sqrt(a*a/16+b*b/4);
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V*a/L/4;
        V_z=-V*b/L/2;
    }

    else if(t<T1+T2+T3+T4){
        L=sqrt(a*a/16+9*b*b/16);
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V*a/L/4;
        V_z=V*3*b/L/4;
    }
    t+=dt;
}


void handWriting::Write_P(){

    double T1=T/5;
    double T2=T/5;
    double T3=T/5;
    double T4=T/5;
    double T5=T/5;


    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){

        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(0,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }

    else if(t<T1+T2+T3){

        L=a/2;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V;
        V_z=0;
    }

    else if(t<T1+T2+T3+T4){
        L=M_PI*b/4;
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V*cos(P/L*M_PI);
        V_z=-V*sin(P/L*M_PI);
    }

    else if(t<T1+T2+T3+T4+T5){

        L=a/2;
        V=Velocity(L,t-T1-T2-T3-T4,T5);
        P=Position(L,t-T1-T2-T3-T4,T5);
        V_x=0;
        V_y=-V;
        V_z=0;
    }

    t+=dt;
}


void handWriting::Write_D(){

    double T1=T/3;
    double T2=T/3;
    double T3=T/3;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){

        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(0,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }

    else if(t<T1+T2+T3){
        L=M_PI*b/2;
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=V*cos(P/L*M_PI);
        V_z=-V*sin(P/L*M_PI);
    }
    t+=dt;
}

void handWriting::Write_K(){

    double T1=T/4;
    double T2=T/4;
    double T3=T/4;
    double T4=T/4;

    if(t<T1){
        L=b;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=-V;
    }
    else if(t<T1+T2){
        V_x=move_to_diff(-d,t,T1,T1+T2/2)-move_to_diff(-d,t,T1+T2/2,T1+T2);
        V_y=move_to_diff(3*a/4,t,T1,T1+T2);
        V_z=move_to_diff(b,t,T1,T1+T2);
    }
    else if(t<T1+T2+T3){

        L=sqrt(9*a*a/16+9*b*b/64);
        V=Velocity(L,t-T1-T2,T3);
        P=Position(L,t-T1-T2,T3);
        V_x=0;
        V_y=-V*3*a/4/L;
        V_z=-V*3*b/8/L;
    }

    else if(t<T1+T2+T3+T4){
        L=sqrt(25*b*b/64+a*a);
        V=Velocity(L,t-T1-T2-T3,T4);
        P=Position(L,t-T1-T2-T3,T4);
        V_x=0;
        V_y=V*a/L;
        V_z=-V*5*b/8/L;
    }
    t+=dt;
}

void handWriting::Write_x_axis(){

    double T1=T;
    if(t<T1){
        L=a_x;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V*a/L/2;
        V_z=V*b/L;
    }
    t+=dt;
}

void handWriting::Write_y_axis(double a_y, double T){

    double T1=T;
    if(t<T1){
        L=a_y;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=V;
        V_z=0;
    }
    t+=dt;
}

void handWriting::Write_x_axis_n(){

    double T1=T;
    if(t<T1){
        L=a_x;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=0;
        V_z=0;
    }
    t+=dt;
}

void handWriting::Write_y_axis_n(double a_y_n, double T){

    double T1=T;
    if(t<T1){
        L=a_y_n;
        V=Velocity(L,t,T1);
        P=Position(L,t,T1);
        V_x=0;
        V_y=-V;
        V_z=0;
    }
    t+=dt;
}


void handWriting::move2next(string current,string next){
    V_x=move_to_diff(-d,t,0,T/2)-move_to_diff(-d,t,T/2,T);
    V_y=move_to_diff(a+c+Y_start[next]-Y_end[current],t,0,T);
    V_z=move_to_diff(Z_start[next]-Z_end[current],t,0,T);
    t+=dt;
}

void handWriting::moveback(){
    V_x=move_to_diff(-d,t,0,T);
    V_y=0;
    V_z=0;
    t+=dt;
}

double handWriting::move2zero(double theta,double t,double T_home){
    double c3=10/pow(T_home,3);
    double c4=-15/pow(T_home,4);
    double c5=6/pow(T_home,5);
    double theta0;
    if(t==0){return theta;}
    if(t>=T_home){return 0;}
    double dt=0.005;
    if(fabs(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5))>1e-6){
        theta0=theta/(1-c3*pow(t-dt,3)-c4*pow(t-dt,4)-c5*pow(t-dt,5));
        // qDebug()<<theta0;
        return theta0*(1-c3*pow(t,3)-c4*pow(t,4)-c5*pow(t,5));
    }
}

