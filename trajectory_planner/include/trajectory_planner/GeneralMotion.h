# pragma once

#include "MinJerk.h"

class GeneralMotion: private MinJerk{
    public:
        GeneralMotion(double dt);
        ~GeneralMotion();
        void changeInPlace(Vector3d init_com_pos, Vector3d final_com_pos, 
                           Vector3d init_com_orient, Vector3d final_com_orient,
                           Vector3d init_lankle_pos, Vector3d final_lankle_pos,
                           Vector3d init_lankle_orient, Vector3d final_lankle_orient,
                           Vector3d init_rankle_pos, Vector3d final_rankle_pos,
                           Vector3d init_rankle_orient, Vector3d final_rankle_orient,
                           double time);

        Vector3d* getCOMPos(){
            return COMPos_;
            }
        Vector3d* getLAnklePos(){
            return LAnklePos_;
            }
        Vector3d* getRAnklePos(){
            return RAnklePos_;
            }
        Matrix3d* getLAnkleOrient(){
            return LAnkleOrient_;
            }
        Matrix3d* getRAnkleOrient(){
            return RAnkleOrient_;
            }
        Matrix3d* getCOMOrient(){
            return COMOrient_;
            }
        int getLength(){
            return length_;
        }

    private:
        int length_;
        double dt_;
        Vector3d* COMPos_;
        Vector3d* LAnklePos_;
        Vector3d* RAnklePos_;
        Matrix3d* COMOrient_;
        Matrix3d* LAnkleOrient_;
        Matrix3d* RAnkleOrient_;
};