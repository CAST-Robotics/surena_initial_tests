#include "FootStepPlanner.h"

FootStepPlanner::FootStepPlanner(double torso) : torso_{torso}
{
    isTurn_ = false;
}

FootStepPlanner::~FootStepPlanner()
{

}

void FootStepPlanner::setParams(double step_length, double step_width, double step_count,
                                double step_height, double step_angle){
    stepLength_ = step_length;
    stepWidth_ = step_width;
    stepCount_ = step_count;
    stepHeight_ = step_height;
    stepRotAngle_ = step_angle;

    if(stepRotAngle_ != 0)
        isTurn_ = true;
    else
        isTurn_ = false;
}

void FootStepPlanner::planSteps(){
    if(isTurn_)
        this->planTurn();
    else
        this->planStraight();
}

void FootStepPlanner::planStraight(){
    int sign;
    if (stepWidth_ == 0)
        sign = 1;
    else
        sign = (stepWidth_/abs(stepWidth_));

    footPrints_.push_back(Vector3d(0.0, torso_ * sign, 0.0));
    footPrints_.push_back(Vector3d(0.0, -torso_ * sign, 0.0));
    for (int i=0; i<2; i++)
        footYaws_.push_back(0.0);

    for(int i = 2; i <= stepCount_ + 1; i++){

        footYaws_.push_back(0.0);

        if (i == 2 || i == stepCount_ + 1)
            footPrints_.push_back(footPrints_[i-2] + Vector3d(stepLength_, stepWidth_, 0.0));
        else
            footPrints_.push_back(footPrints_[i-2] + Vector3d(2 * stepLength_, stepWidth_, 0.0));
    }
}

void FootStepPlanner::planTurn(){
    double r = abs(stepLength_/stepRotAngle_);
    int sign = abs(stepLength_) / stepLength_;

    footPrints_.push_back(Vector3d(0.0, -sign * torso_, 0.0));
    footYaws_.push_back(0.0);
    
    for (int i=1; i<=stepCount_; i++){
        Vector3d temp = (r + pow(-1, i-1) * torso_) * Vector3d(sin(stepRotAngle_ * (i-1)), sign * cos(stepRotAngle_ * (i-1)), 0.0) + Vector3d(0.0, -sign * r, 0.0);
        footPrints_.push_back(temp);
        footYaws_.push_back(-sign * stepRotAngle_ * (i-1));
    }

    footPrints_.push_back((r + pow(-1, stepCount_) * torso_) * Vector3d(sin(stepRotAngle_ * (stepCount_-1)), sign * cos(stepRotAngle_ * (stepCount_-1)), 0.0) + Vector3d(0.0, -sign * r, 0.0));
    footYaws_.push_back(-sign * stepRotAngle_ * (stepCount_-1));
}
