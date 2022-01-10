#include "../include/trajectory_planner/Link.h"

_Link::_Link(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link* parent){
    this->ID_ = ID;
    this->parent_ = parent;
    this->a_ = a;
    this->b_ = b;
    this->m_ = m;
    this->I_ = inertia;

    this->q_ = 0.0;
    this->dq_ = 0.0;
    this->ddq_ = 0.0;

 }

 _Link::_Link(const _Link& source){
    ID_ = source.ID_;
    parent_ = source.parent_;
    a_ = source.a_;
    b_ = source.b_;
    m_ = source.m_;
    I_ = source.I_;

    q_ = 0.0;
    dq_ = 0.0;
    ddq_ = 0.0;
}

 short int _Link::getID(){
    return this->ID_;
 }

double _Link::q(){
    return this->q_;
}

double _Link::dq(){
    return this->dq_;
}

void _Link::update(double q, double dq, double ddq){
    this->q_ = q;
    this->dq_ = dq;
    this->ddq_ = ddq;
}

Vector3d _Link::getPose(){
    return this->p_;
}

Matrix3d _Link::getRot(){
    return this->R_;
}

_Link* _Link::getParent(){
    return this->parent_;
}

void _Link::initPose(Vector3d p, Matrix3d r){
	this->p_ = p;
	this->R_ = r;
}

void _Link::setParams(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link* parent){
    this->ID_ = ID;
    this->parent_ = parent;
    this->a_ = a;
    this->b_ = b;
    this->m_ = m;
    this->I_ = inertia;

    this->q_ = 0.0;
    this->dq_ = 0.0;
    this->ddq_ = 0.0;
}

Matrix3d _Link::rodrigues(Vector3d w, double dt){
    // Helper Function for calculating attitude in Forward Kinematics

    if (w.norm() < numeric_limits<double>::epsilon()){
        return Matrix3d::Identity(3,3);
    }
    else{
        Vector3d wn = w/w.norm();
        double th = w.norm() * dt;
        Matrix3d w_wedge;
        w_wedge << 0.0, -wn(2), wn(1),
                   wn(2), 0.0, -wn(0),
                   -wn(1), wn(0), 0.0;
        Matrix3d R = Matrix3d::Identity(3,3) + w_wedge * sin(th) + w_wedge * w_wedge * (1 - cos(th));
        return R;
    }
}

MatrixXd _Link::FK(){
    if(this->parent_ == NULL){
        return this->transformation();
    }
    else{
        this->parent_->FK();
        this->p_ = this->parent_->getRot() * this->b_ + this->parent_->p_;
        this->R_ = this->parent_->getRot() * this->rodrigues(this->a_, this->q_);
        return this->transformation();
    }
}

MatrixXd _Link::transformation(){
    // returns homogeneous transformation matrix
    MatrixXd T(4,4);
    T << this->R_(0,0), this->R_(0,1), this->R_(0,2), this->p_(0),
         this->R_(1,0), this->R_(1,1), this->R_(1,2), this->p_(1),
         this->R_(2,0), this->R_(2,1), this->R_(2,2), this->p_(2),
         0.0, 0.0, 0.0, 1.0;
    return T;
}

MatrixXd _Link::updateJacobian(){
    vector<_Link*> idx;
    _Link *base = this;
    //cout << this->getRot() << endl;
    while(base->getID() != 0){
        idx.push_back(base);
        base = base->getParent();
    }
    //cout << "Route:\n";
    //for (int i = 0; i < idx.size(); i++){
    //    cout << idx[i]->getID() << "-->";
    //}
    //cout << endl;
    MatrixXd jacobian =  MatrixXd::Zero(6,idx.size());
    Vector3d target = this->getPose();
    for(int n = idx.size() - 1; n >= 0; n--){
        Vector3d a = idx[n]->getRot() * idx[n]->a_;
        jacobian.block<3,1>(0,idx.size() - 1 - n) = a.cross(target - idx[n]->getPose());
        jacobian.block<3,1>(3,idx.size() - 1 - n) = a;
    }

    return jacobian;
}

MatrixXd _Link::getVel(){
    MatrixXd jacobian = this->updateJacobian();
    int rows = jacobian.cols();
    MatrixXd dq (rows, 1);
    _Link *link = this; 
    for(int i = rows - 1; i >= 0; i--)
    {
        dq(i, 0) = link->dq();
        link = link->getParent();
    }
    return jacobian * dq;
}

_Link::~_Link(){

}

/*
int main(){
    Vector3d a[12];
	Vector3d b[12];
	// Defining Joint Axis
	a[0] << 0.0, 0.0, 1.0;
	a[1] << 1.0, 0.0, 0.0;
	a[2] << 0.0, 1.0, 0.0;
	a[3] << 0.0, 1.0, 0.0;
	a[4] << 0.0, 1.0, 0.0;
	a[5] << 1.0, 0.0, 0.0;
	a[6] = a[0];
	a[7] = a[1];
	a[8] = a[2];
	a[9] = a[3];
	a[10] = a[4];
	a[11] = a[5];
	// Defining Joint Positions
	b[0] << 0.0, -0.115, 0.0;
	b[1] << 0.0, 0.0, 0.0;
	b[2] = b[1];
	b[3] << 0.0, 0.0, -0.36;
	b[4] << 0.0, 0.0, -0.37;
	b[5] = b[1];
	b[6] = -b[0];
	b[7] = b[1];
	b[8] = b[2];
	b[9] = b[3];
	b[10] = b[4];
	b[11] = b[5];


	_Link pelvis(0, Vector3d::Ones(3), Vector3d::Ones(3), 3.0, Matrix3d::Identity(3,3));
	Vector3d position(0.0, 0.0, 0.73);
	pelvis.initPose(position, Matrix3d::Identity(3, 3));

	_Link rHipY(1, a[0], b[0], 3.0, Matrix3d::Identity(3, 3), &pelvis);
	_Link rHipR(2, a[1], b[1], 3.0, Matrix3d::Identity(3, 3), &rHipY);
	_Link rHipP(3, a[2], b[2], 3.0, Matrix3d::Identity(3, 3), &rHipR);
	_Link rKnee(4, a[3], b[3], 3.0, Matrix3d::Identity(3, 3), &rHipP);
	_Link rAnkleP(5, a[4], b[4], 3.0, Matrix3d::Identity(3, 3), &rKnee);
	_Link rAnkleR(6, a[5], b[5], 3.0, Matrix3d::Identity(3, 3), &rAnkleP);
    rHipP.update(-M_PI/6, 0, 0);
    rKnee.update(M_PI/3, 0, 0);
    rAnkleP.update(-M_PI/6, 0, 0);
	_Link lHipY(7, a[6], b[6], 3.0, Matrix3d::Identity(3, 3), &pelvis);
	_Link lHipR(8, a[7], b[7], 3.0, Matrix3d::Identity(3, 3), &lHipY);
	_Link lHipP(9, a[8], b[8], 3.0, Matrix3d::Identity(3, 3), &lHipR);
	_Link lKnee(10, a[9], b[9], 3.0, Matrix3d::Identity(3, 3), &lHipP);
	_Link lAnkleP(11, a[10], b[10], 3.0, Matrix3d::Identity(3, 3), &lKnee);
	_Link lAnkleR(12, a[11], b[11], 3.0, Matrix3d::Identity(3, 3), &lAnkleP);


	cout << rAnkleR.FK() << endl << "---------\n";
	cout << rAnkleR.updateJacobian() << endl;
    cout << rAnkleR.getVel() << endl;

    return 0;
}*/
