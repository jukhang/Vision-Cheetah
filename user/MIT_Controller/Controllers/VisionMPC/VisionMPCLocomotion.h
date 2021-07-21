#ifndef CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include "cppTypes.h"
#include <lcm-cpp.hpp>
using Eigen::Array4f;
using Eigen::Array4i;


class VisionGait
{
public:
    VisionGait(int nMPC_segments, Vec4<int> offsets, Vec4<int>  durations, const std::string& name="");
    ~VisionGait();
    Vec4<float> getContactState();
    Vec4<float> getSwingState();
    int* mpc_gait();
    void setIterations(int iterationsPerMPC, int currentIteration);
    int _stance;
    int _swing;


private:
    int _nMPC_segments;
    int* _mpc_table;
    Array4i _offsets; // offset in mpc segments
    Array4i _durations; // duration of step in mpc segments
    Array4f _offsetsFloat; // offsets in phase (0 to 1)
    Array4f _durationsFloat; // durations in phase (0 to 1)
    int _iteration;
    int _nIterations;
    float _phase;


};


class VisionMPCLocomotion {
public:
    VisionMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
    void initialize();

//  template<typename T>
//  void run(ControlFSMData<T>& data,
//      const Vec3<T> & vel_cmd, const DMat<T> & height_map, const DMat<int> & idx_map);
    template<typename T>
    void run(ControlFSMData<T>& data,
             const Vec3<T> & vel_cmd, const DMat<T> & height_map, const DMat<float> & idx_map);//1119
    Vec3<float> pBody_des;
    Vec3<float> vBody_des;
    Vec3<float> aBody_des;

    Vec3<float> pBody_RPY_des;
    Vec3<float> vBody_Ori_des;

    Vec3<float> pFoot_des[4];
    Vec3<float> vFoot_des[4];
    Vec3<float> aFoot_des[4];

    Vec3<float> Fr_des[4];

    Vec4<float> contact_state;

private:
    void _UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,
                            const DMat<float> & height_map, const DMat<int> & idx_map);
    void _IdxMapCheckingOld(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected,
                          const DMat<int> & idx_map);
    //1119
    void _UpdateFootholdNew(Vec3<float> & foot_des, Vec3<float> & foot_ini,const Vec3<float> & body_pos,
                            const DMat<float> & height_map, const DMat<float> & idx_map);
    void _IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected,
                         const DMat<float> & height_map, const DMat<float> & idx_map,Vec3<float> & foot_ini);
    bool _SurroundingChecking(int x_idx, int y_idx, const DMat<float> & idx_map);
    bool _FeasibilityChecking(int x_idx, int y_idx, const DMat<float> & height_map,Vec3<float> & foot_ini);
    Vec3<float> _fin_foot_loc[4];
    float grid_size = 0.02;
    int xsize=100,ysize=100,half_ysize=50;//1119
    Vec3<float> v_des_world;
    Vec3<float> rpy_des;
    Vec3<float> v_rpy_des;

//    float _body_height = 0.29;
    void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data);
    void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
    int iterationsBetweenMPC;
    int horizonLength;
    float dt;
    float dtMPC;
    int iterationCounter = 0;
    Vec3<float> f_ff[4];
    Vec4<float> swingTimes;
    FootSwingTrajectory<float> footSwingTrajectories[4];
    VisionGait trotting, standing;
    Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
    bool firstRun = true;
    bool firstSwing[4];
    float swingTimeRemaining[4];
    float stand_traj[6];
    int current_gait;
    int gaitNumber;
    static const int sumStep = 18;//0528

    Vec3<float> world_position_desired;
    Vec3<float> rpy_int;
    Vec3<float> rpy_comp;
    Vec3<float> pFoot[4];
    float pzFootMin;
    Vec3<float> pfoot_des[4];//1111
    Vec3<float> pfoot_ini[4];//1111
    Vec3<float> pfoot_log[4];//1123
    Vec3<float> Pf[4];//1112
    Vec3<float> pDesFootWorld[4];//1112
    float trajAll[12*36];
    lcm::LCM footLCM;//1112
    float vx=0.21;//1113
    float vy=0.;//0.0208;
    Vec3<float> delta = {0.1,0.00,0.00};
    vector<vector<float> > z_list; //initialization is in function initialize();
    int stepcount[4] = {0,0,0,0};//1114
    float BodyHeight = 0.29;
    bool checkStep_OK = true;//1119
    float StepHeight;
    constexpr static const float camera_offset = 0.20;
    int getxidx(float pfootx);
    int getyidx(float pfooty);

    MIT_UserParameters* _parameters = nullptr;

};


#endif //CHEETAH_SOFTWARE_VISION_MPCLOCOMOTION_H
