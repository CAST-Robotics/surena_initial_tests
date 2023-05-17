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

        int* getRobotState(){
            return robotState_;
        }

        void _changeInPlace(Vector3d init_com_pos, Vector3d final_com_pos, 
                           Vector3d init_com_orient, Vector3d final_com_orient,
                           Vector3d init_lankle_pos, Vector3d final_lankle_pos,
                           Vector3d init_lankle_orient, Vector3d final_lankle_orient,
                           Vector3d init_rankle_pos, Vector3d final_rankle_pos,
                           Vector3d init_rankle_orient, Vector3d final_rankle_orient,
                           double time);

        vector<Vector3d> _getCoMPos(){
            return _CoMPos_;
            }
        vector<Vector3d> _getLAnklePos(){
            return _LAnklePos_;
            }
        vector<Vector3d> _getRAnklePos(){
            return _RAnklePos_;
            }
        vector<Matrix3d> _getLAnkleOrient(){
            return _LAnkleOrient_;
            }
        vector<Matrix3d> _getRAnkleOrient(){
            return _RAnkleOrient_;
            }
        vector<Matrix3d> _getCoMOrient(){
            return _CoMOrient_;
            }
        vector<int> _getRobotState(){
            return _robotState_;
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

        //Robot Movment State Indicator (0:Stance, 1:Double Support, 2:Right Single Support, 3: Left Single Support, 4:None)
        int* robotState_;

        vector<Vector3d> _CoMPos_;
        vector<Vector3d> _LAnklePos_;
        vector<Vector3d> _RAnklePos_;

        vector<Matrix3d> _CoMOrient_;
        vector<Matrix3d> _LAnkleOrient_;
        vector<Matrix3d> _RAnkleOrient_;

        //Robot Movment State Indicator (0:Stance, 1:Double Support, 2:Right Single Support, 3: Left Single Support, 4:None)
        vector<int> _robotState_;
};