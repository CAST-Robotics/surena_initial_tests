#pragma once

#include "MinJerk.h"

class FootStepPlanner : public MinJerk {
    public:
        FootStepPlanner(double torso);
        ~FootStepPlanner();
        void setParams(double step_length, double step_width, double step_count,
                       double step_height=0, double step_angle=0);
        void planSteps();
        void planStraight();
        void planTurn();

        inline vector<Vector3d> getFootPrints() {return footPrints_;}
        inline vector<double> getFootYaws(){return footYaws_;}

    private:
        double stepWidth_;
        double stepLength_;
        double stepHeight_;
        double stepRotAngle_;
        int stepCount_;
        double torso_; //half of the distance between two soles in home state
        bool isTurn_;

        vector<Vector3d> footPrints_;
        vector<double> footYaws_;

};