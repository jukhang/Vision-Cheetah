#include <iostream>
#include <Utilities/Utilities_print.h>
#include "VisionMPCLocomotion.h"
#include "VisionMPC_interface.h"
#include "lcm-cpp.hpp"
#include "footsteps.hpp"

///////////////
///   GAIT  ///
///////////////
VisionGait::VisionGait(int nMPC_segments, Vec4<int> offsets,
                       Vec4<int> durations, const std::string &name) :
        _offsets(offsets.array()),
        _durations(durations.array()),
        _nIterations(nMPC_segments)
{
    _mpc_table = new int[nMPC_segments * 4];//four legs

    _offsetsFloat = offsets.cast<float>() / (float) nMPC_segments;
    _durationsFloat = durations.cast<float>() / (float) nMPC_segments;
    std::cout << "VisionGait " << name << "\n";
    std::cout << "nMPC_segments    : " << _nIterations << "\n";
    std::cout << "offsets (int)    : " << _offsets.transpose() << "\n";
    std::cout << "durations (int)  : " << _durations.transpose() << "\n";
    std::cout << "offsets (float)  : " << _offsetsFloat.transpose() << "\n";
    std::cout << "durations (float): " << _durationsFloat.transpose() << "\n";
    std::cout << "\n\n";

    _stance = durations[0];
    _swing = nMPC_segments - durations[0];

}


VisionGait::~VisionGait() {
    delete[] _mpc_table;
}


Vec4<float> VisionGait::getContactState() {
    Array4f progress = _phase - _offsetsFloat;//offset=0,progress=phase

    for(int i = 0; i < 4; i++)
    {
        if(progress[i] < 0) progress[i] += 1.;//phase<_offsetsFloat
        if(progress[i] > _durationsFloat[i])
        {
            progress[i] = 0.;
        }
        else
        {
            progress[i] = progress[i] / _durationsFloat[i];
        }
    }
    return progress.matrix();
}

Vec4<float> VisionGait::getSwingState() {
    Array4f swing_offset = _offsetsFloat + _durationsFloat;
    for(int i = 0; i < 4; i++)
        if(swing_offset[i] > 1) swing_offset[i] -= 1.;
    Array4f swing_duration = 1. - _durationsFloat;

    Array4f progress = _phase - swing_offset;

    for(int i = 0; i < 4; i++)
    {
        if(progress[i] < 0) progress[i] += 1.f;
        if(progress[i] > swing_duration[i])
        {
            progress[i] = 0.;
        }
        else
        {
            progress[i] = progress[i] / swing_duration[i];
        }
    }

    return progress.matrix();
}

int* VisionGait::mpc_gait() {
    for(int i = 0; i < _nIterations; i++)
    {
        int iter = (i + _iteration + 1) % _nIterations;
        Array4i progress = iter - _offsets;
        for(int j = 0; j < 4; j++)
        {
            if(progress[j] < 0) progress[j] += _nIterations;
            if(progress[j] < _durations[j])
                _mpc_table[i*4 + j] = 1;//stance
            else
                _mpc_table[i*4 + j] = 0;//swing
        }
    }

    return _mpc_table;
}

void VisionGait::setIterations(int iterationsPerMPC, int currentIteration) {
    _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
    _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
}


////////////////////
///  Controller  ///
////////////////////

VisionMPCLocomotion::VisionMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
        iterationsBetweenMPC(_iterations_between_mpc),
        horizonLength(16),
        dt(_dt),
        trotting(horizonLength, Vec4<int>(0,8,8,0), Vec4<int>(8,8,8,8),"Trotting"),
//          trotting(horizonLength, Vec4<int>(0,9,9,0), Vec4<int>(9,9,9,9),"Trotting"),
        standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(16,16,16,16),"Standing")
{
    _parameters = parameters;
    dtMPC = dt * iterationsBetweenMPC;
    printf("[Vision MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",
           dt, iterationsBetweenMPC, dtMPC);
    vision_setup_problem(dtMPC, horizonLength, 0.4, 120);
    rpy_comp[0] = 0;
    rpy_comp[1] = 0;
    rpy_comp[2] = 0;
    rpy_int[0] = 0;
    rpy_int[1] = 0;
    rpy_int[2] = 0;
    StepHeight = parameters->Swing_traj_height;
    for(int i = 0; i < 4; i++)
        firstSwing[i] = true;
}

void VisionMPCLocomotion::initialize(){
    for(int i = 0; i < 4; i++) firstSwing[i] = true;
    firstRun = true;
    rpy_des.setZero();
    v_rpy_des.setZero();
//  initialize footsteps
//5cm
/*    for (int t=0;t<7;t++) z_list.push_back({0.05,0.05,0.0,0.0});
    for (int t=7;t<10;t++) z_list.push_back({0.05,0.05,0.05,0.05});
    for (int t=10;t<18;t++) z_list.push_back({0.0,0.0,0.05,0.05});
    for (int t=18;t<20;t++) z_list.push_back({0.0,0.0,0.0,0.0});*/
//10cm
/*    for (int t=0;t<4;t++) z_list.push_back({0.05,0.05,0.0,0.0});
    for (int t=4;t<6;t++) z_list.push_back({0.05,0.05,0.05,0.05});
    for (int t=6;t<9;t++) z_list.push_back({0.0,0.0,0.05,0.05});
    for (int t=9;t<20;t++) z_list.push_back({0.0,0.0,0.0,0.0});
    for (int t=0;t<20;t++) std::cout<<z_list[t][0]<<z_list[t][1]<<z_list[t][2]<<z_list[t][3]<<std::endl;*/
//3stairs-10-5cm
/*    for (int t=0;t<3;t++) z_list.push_back({0.05,0.05,0.0,0.0});
    for (int t=3;t<5;t++) z_list.push_back({0.1,0.1,0.0,0.0});
    for (int t=5;t<6;t++) z_list.push_back({0.15,0.15,0.05,0.05});
    for (int t=6;t<8;t++) z_list.push_back({0.15,0.15,0.1,0.10});
    for (int t=8;t<20;t++) z_list.push_back({0.15,0.15,0.15,0.15});*/
//3stairs-10-4cm
/*    for (int t=0;t<3;t++) z_list.push_back({0.04,0.04,0.0,0.0});
    for (int t=3;t<5;t++) z_list.push_back({0.08,0.08,0.0,0.0});
    for (int t=5;t<6;t++) z_list.push_back({0.12,0.12,0.04,0.04});
    for (int t=6;t<8;t++) z_list.push_back({0.12,0.12,0.08,0.08});
    for (int t=8;t<20;t++) z_list.push_back({0.12,0.12,0.12,0.12});*/
//3stairs-10-6cm
/*    for (int t=0;t<2;t++) z_list.push_back({0.06,0.06,0.0,0.0});
    for (int t=2;t<4;t++) z_list.push_back({0.12,0.12,0.0,0.0});
    for (int t=4;t<7;t++) z_list.push_back({0.18,0.18,0.06,0.06});
    for (int t=7;t<9;t++) z_list.push_back({0.18,0.18,0.12,0.12});
    for (int t=9;t<20;t++) z_list.push_back({0.18,0.18,0.18,0.18});
    for (int t=0;t<20;t++) std::cout<<z_list[t][0]<<z_list[t][1]<<z_list[t][2]<<z_list[t][3]<<std::endl;*/
//3stairs-5
/*    for (int t=0;t<3;t++) z_list.push_back({0.05,0.05,0.0,0.0});
    for (int t=3;t<6;t++) z_list.push_back({0.1,0.1,0.0,0.0});
    for (int t=6;t<7;t++) z_list.push_back({0.15,0.15,0.0,0.0});
    for (int t=7;t<10;t++) z_list.push_back({0.15,0.15,0.05,0.05});
    for (int t=10;t<13;t++) z_list.push_back({0.15,0.15,0.1,0.1});
    for (int t=13;t<20;t++) z_list.push_back({0.15,0.15,0.15,0.15});*/

}

void VisionMPCLocomotion::_UpdateFoothold(Vec3<float> & foot_pos, const Vec3<float> & body_pos,
    const DMat<float> & height_map, const DMat<int> & idx_map){

    Vec3<float> local_pf = foot_pos - body_pos;

    int row_idx_half = height_map.rows()/2;
    int col_idx_half = height_map.rows()/2;

    int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;

    int x_idx_selected = x_idx;
    int y_idx_selected = y_idx;

    _IdxMapCheckingOld(x_idx, y_idx, x_idx_selected, y_idx_selected, idx_map);

    foot_pos[0] = (x_idx_selected - row_idx_half)*grid_size + body_pos[0];
    foot_pos[1] = (y_idx_selected - col_idx_half)*grid_size + body_pos[1];
    foot_pos[2] = height_map(x_idx_selected, y_idx_selected);

}

void VisionMPCLocomotion::_IdxMapCheckingOld(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected,
                                          const DMat<int> & idx_map){

    if(idx_map(x_idx, y_idx) == 0){ // (0,0)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx+1, y_idx) == 0){ // (1, 0)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx+1, y_idx+1) == 0){ // (1, 1)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx, y_idx+1) == 0){ // (0, 1)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx-1, y_idx+1) == 0){ // (-1, 1)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx-1, y_idx) == 0){ // (-1, 0)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx-1, y_idx-1) == 0){ // (-1, -1)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx, y_idx-1) == 0){ // (0, -1)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+1, y_idx-1) == 0){ // (1, -1)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+2, y_idx-1) == 0){ // (2, -1)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+2, y_idx) == 0){ // (2, 0)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx+2, y_idx+1) == 0){ // (2, 1)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx+2, y_idx+2) == 0){ // (2, 2)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx+2;

    }else{
        printf("no proper step location (%d, %d)\n", x_idx, y_idx);
        x_idx_selected = x_idx;
        y_idx_selected = y_idx;
    }
}

/*
 * Get the foot position's map x index
 * */
int VisionMPCLocomotion::getxidx(float pfootx){

    pfootx=pfootx-camera_offset;
    int x_idx = xsize-floor(pfootx/grid_size) ;
    return x_idx;
}
/*
 * Get the foot position's map y index
 * */
int VisionMPCLocomotion::getyidx(float pfooty){

    int y_idx = half_ysize-floor(pfooty/grid_size) ;
    return y_idx;
}
void VisionMPCLocomotion::_UpdateFootholdNew(Vec3<float> & foot_des, Vec3<float> & foot_ini,const Vec3<float> & body_pos,
                                             const DMat<float> & height_map, const DMat<float> & idx_map)
{
    Vec3<float> pf = foot_des - body_pos;pf[0]=0.;//useless

    Vec3<float> foot_ini_tmp;
    foot_ini_tmp=foot_ini;

    foot_ini_tmp[0]=foot_ini_tmp[0]-camera_offset; //foot pos relative to camera

    int x_idx = getxidx(foot_des[0]) ;
    int y_idx = getyidx(foot_des[1]) ;

    int x_idx_selected = x_idx;
    int y_idx_selected = y_idx;

    if (x_idx_selected<98 && x_idx_selected>1){
        _IdxMapChecking(x_idx, y_idx, x_idx_selected, y_idx_selected, height_map, idx_map,foot_ini_tmp);
        foot_des[0] = (xsize-x_idx_selected)*grid_size + camera_offset;
        foot_des[1] = (half_ysize-y_idx_selected)*grid_size ;
        if (height_map(x_idx_selected,y_idx_selected)>=0.02){
            foot_des[2] = height_map(x_idx_selected, y_idx_selected) - 0.003;
        }
        else
            foot_des[2] = -0.003;
    }
    else {
        //do nothing
        foot_des[0] = foot_des[0];// + camera_offset;
    }

}
/*
 * Check if the surrounding area of a selected foothold is good too.
 * */
bool VisionMPCLocomotion::_SurroundingChecking(int x_idx, int y_idx, const DMat<float> & idx_map) {//1123

    float s01,s10,s11,s1_1,s0_1,s_10,s_11,s_1_1;
    bool surrounding_OK=false;

    s10 =idx_map(x_idx-1, y_idx);
    s1_1=idx_map(x_idx-1, y_idx-1);
    s11 =idx_map(x_idx-1, y_idx+1);
    s01 =idx_map(x_idx, y_idx+1);
    s0_1=idx_map(x_idx, y_idx-1);
    s_10=idx_map(x_idx+1, y_idx);
    s_11=idx_map(x_idx+1, y_idx+1);
    s_1_1=idx_map(x_idx+1, y_idx-1);

    if ( s01<= 1.5 && s0_1<=1.5 && s10<=1.5 && s11<=1.5 && s1_1<=1.5 && s_10<=1.5 && s_11<=1.5 && s_1_1<=1.5){
        surrounding_OK=true;
    }
    else {
        surrounding_OK=false;
    }
    return surrounding_OK;
}
/*
 * Check if the difference of two steps is too great to cover.
 * */
bool VisionMPCLocomotion::_FeasibilityChecking(int x_idx, int y_idx, const DMat<float> & height_map, Vec3<float> & foot_ini) {//1123

    bool feasibility_OK=true;
    int x_idx_last = xsize-floor(foot_ini[0]/grid_size) ;
    int y_idx_last = ysize/2-floor(foot_ini[1]/grid_size) ;

    if (height_map(x_idx,y_idx)>height_map(x_idx_last,y_idx_last) && height_map(x_idx,y_idx)-height_map(x_idx_last,y_idx_last)>=StepHeight*.7 ){
        feasibility_OK=false;
        std::cout<<" current: "<<height_map(x_idx_last,y_idx_last)<<"next: "<<height_map(x_idx,y_idx)<<std::endl;
    }
    else if (height_map(x_idx,y_idx)<height_map(x_idx_last,y_idx_last) && height_map(x_idx_last,y_idx_last)-height_map(x_idx,y_idx)>=StepHeight*.5 ){
        feasibility_OK=false;
        std::cout<<" current: "<<height_map(x_idx_last,y_idx_last)<<"next: "<<height_map(x_idx,y_idx)<<std::endl;
    }
    else feasibility_OK=true;
    return feasibility_OK;
}

/*
 * Check if the traversability score of the selected foothold is good
 * as well as the results of Feasibility and Surrounding checking.
 * */
void VisionMPCLocomotion::_IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected,
                                          const DMat<float> & height_map, const DMat<float> & idx_map, Vec3<float> & foot_ini){//1119
    checkStep_OK=true;
    if(idx_map(x_idx, y_idx) <= 1.5 && _FeasibilityChecking(x_idx,y_idx,height_map,foot_ini) && _SurroundingChecking(x_idx,y_idx,idx_map) ){ // (0,0)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx+1, y_idx)<=1.5 && _FeasibilityChecking(x_idx+1,y_idx,height_map,foot_ini) && _SurroundingChecking(x_idx+1,y_idx,idx_map)){ // (1, 0)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx-1, y_idx)<=1.5 && _FeasibilityChecking(x_idx-1,y_idx,height_map,foot_ini) && _SurroundingChecking(x_idx-1,y_idx,idx_map)){ // (-1, 0)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx, y_idx+1)<=1.5 && _FeasibilityChecking(x_idx,y_idx+1,height_map,foot_ini) && _SurroundingChecking(x_idx,y_idx+1,idx_map)){ // (0, 1)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx, y_idx-1) <= 1.5 && _FeasibilityChecking(x_idx,y_idx-1,height_map,foot_ini) && _SurroundingChecking(x_idx,y_idx-1,idx_map)){ // (0, -1)
        x_idx_selected = x_idx;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+1, y_idx-1) <= 1.5 && _FeasibilityChecking(x_idx+1,y_idx-1,height_map,foot_ini) && _SurroundingChecking(x_idx+1,y_idx-1,idx_map)){ // (1, -1)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+1, y_idx+1) <= 1.5 && _FeasibilityChecking(x_idx+1,y_idx+1,height_map,foot_ini) && _SurroundingChecking(x_idx+1,y_idx+1,idx_map)){ // (1, 1)
        x_idx_selected = x_idx+1;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx-1, y_idx+1) <= 1.5 && _FeasibilityChecking(x_idx-1,y_idx+1,height_map,foot_ini) && _SurroundingChecking(x_idx-1,y_idx+1,idx_map)){ // (-1, 1)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx-1, y_idx-1) <= 1.5 && _FeasibilityChecking(x_idx-1,y_idx-1,height_map,foot_ini) && _SurroundingChecking(x_idx-1,y_idx-1,idx_map)){ // (-1, -1)
        x_idx_selected = x_idx-1;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx-2, y_idx) <= 1.5 && _FeasibilityChecking(x_idx-2,y_idx,height_map,foot_ini) && _SurroundingChecking(x_idx-2,y_idx,idx_map)){ // (-2, 0)
        x_idx_selected = x_idx-2;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx-2, y_idx-1) <= 1.5 && _FeasibilityChecking(x_idx-2,y_idx-1,height_map,foot_ini) && _SurroundingChecking(x_idx-2,y_idx-1,idx_map)){ // (-2, -1)
        x_idx_selected = x_idx-2;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx-2, y_idx+1) <= 1.5 && _FeasibilityChecking(x_idx-2,y_idx+1,height_map,foot_ini) && _SurroundingChecking(x_idx-2,y_idx+1,idx_map)){ // (-2, 1)
        x_idx_selected = x_idx-2;
        y_idx_selected = y_idx+1;

    }else if(idx_map(x_idx+2, y_idx) <= 1.5 && _FeasibilityChecking(x_idx+2,y_idx,height_map,foot_ini) && _SurroundingChecking(x_idx+2,y_idx,idx_map)){ // (2, 0)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx;

    }else if(idx_map(x_idx+2, y_idx-1) <= 1.5 && _FeasibilityChecking(x_idx+2,y_idx-1,height_map,foot_ini) && _SurroundingChecking(x_idx+2,y_idx-1,idx_map)){ // (2, -1)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx-1;

    }else if(idx_map(x_idx+2, y_idx+1) <= 1.5 && _FeasibilityChecking(x_idx+2,y_idx+1,height_map,foot_ini) && _SurroundingChecking(x_idx+2,y_idx+1,idx_map)){ // (2, 1)
        x_idx_selected = x_idx+2;
        y_idx_selected = y_idx+1;

//    }else if(idx_map(x_idx-3, y_idx) <= 1.5 && _SurroundingChecking(x_idx-3,y_idx,idx_map)){ // (-3, 0)
//        x_idx_selected = x_idx-3;
//        y_idx_selected = y_idx;
//    }else if(idx_map(x_idx-3, y_idx-1) <= 1.5 && _SurroundingChecking(x_idx,y_idx,idx_map)){ // (-3, -1)
//        x_idx_selected = x_idx-3;
//        y_idx_selected = y_idx-1;
//    }else if(idx_map(x_idx-3, y_idx+1) <= 1.5 && _SurroundingChecking(x_idx,y_idx,idx_map)){ // (-3, 1)
//        x_idx_selected = x_idx-3;
//        y_idx_selected = y_idx+1;

    }else{ //if cannot find proper footstep, use the step having the best traversablility score
        printf("No proper step in (%d, %d), ", x_idx, y_idx);
        float minscore=idx_map(x_idx,y_idx);
        int min_x=x_idx,min_y=y_idx;

        for (int i=-2;i<=2;i++){
            for (int j = -1; j <= 1; ++j) {
                if(idx_map(x_idx+i,y_idx+j)<minscore){
                    minscore=idx_map(x_idx+i,y_idx+j);
                    min_x=x_idx + i;
                    min_y=y_idx + j;
                }
            }
        }
        
        // for (int i=x_idx-2;i<=x_idx+2;i++){
        //     for (int j = y_idx-1; j <= y_idx+1; ++j) {
        //         if(idx_map(i,j)<minscore){
        //             minscore=idx_map(i,j);
        //             min_x=i;
        //             min_y=j;
        //         }
        //     }
        // }
        x_idx_selected = min_x;
        y_idx_selected = min_y;
        printf("Final select (%d, %d):score %f\n\n", x_idx_selected, y_idx_selected,idx_map(x_idx_selected,y_idx_selected));
        checkStep_OK=false;
    }
}

/*
 * Run the robot vision MPC controller
 * */
template<>
void VisionMPCLocomotion::run(ControlFSMData<float>& data, const Vec3<float> & vel_cmd, const DMat<float> & height_map, const DMat<float> & idx_map) {//1119
//    const Vec3<float> & vel_cmd, const DMat<float> & height_map, const DMat<int> & idx_map) {
    (void)idx_map;

    if(data.controlParameters->use_rc ){
        data.userParameters->cmpc_gait = data._desiredStateCommand->rcCommand->variable[0];
    }

    gaitNumber = data.userParameters->cmpc_gait;
    auto& seResult = data._stateEstimator->getResult();
    if (stepcount[2] >= sumStep) gaitNumber=4;//1124
    // Check if transition to standing
    if(((gaitNumber == 4) && current_gait != 4) || firstRun)
    {
//        if (std::abs(pfoot_ini[0][2]-pfoot_ini[3][2])<=0.01) BodyHeight=0.29+pfoot_ini[0][2];//1124
        stand_traj[0] = seResult.position[0];
        stand_traj[1] = seResult.position[1];
        stand_traj[2] = BodyHeight;//0.29;
        stand_traj[3] = 0;
        stand_traj[4] = 0;
        stand_traj[5] = seResult.rpy[2];
        world_position_desired[0] = stand_traj[0];
        world_position_desired[1] = stand_traj[1];
    }

    // pick gait
    VisionGait* gait = &trotting;
    if(gaitNumber == 1)         gait = &trotting;
    else if(gaitNumber == 4)    gait = &standing;
    current_gait = gaitNumber;

    // integrate position setpoint
    v_des_world[0] = vel_cmd[0];
    v_des_world[1] = vel_cmd[1];
    v_des_world[2] = 0.;
    rpy_des[1] = 0.;
    rpy_des[2] = 0.;//seResult.rpy[2];
    v_rpy_des[2] = vel_cmd[2];
    Vec3<float> v_robot = seResult.vWorld;


    //Integral-esque pitche and roll compensation
    if(fabs(v_robot[0]) > .2) {  //avoid dividing by zero
        rpy_int[1] += dt*(rpy_des[1] - seResult.rpy[1])/v_robot[0];
    }
    if(fabs(v_robot[1]) > 0.1) {
        rpy_int[0] += dt*(rpy_des[0] - seResult.rpy[0])/v_robot[1];
    }

    rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
    rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
    rpy_comp[1] = v_robot[0] * rpy_int[1];
    rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


    for(int i = 0; i < 4; i++) {
        pFoot[i] = seResult.position +                                                //pos in world frame
                   seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) +//pos in body frame
                                                 data._legController->datas[i].p);//pos in hip frame
//                                                 std::cout<<pFoot[i]<<std::endl;
    }

    if(gait != &standing) {
        world_position_desired += dt * Vec3<float>(vx, vy, 0);

//        int x_ini_idx=getxidx(pfoot_ini[0][0]);
//        int y_ini_idx=getyidx(pfoot_ini[0][1]);
////      set body height
////      std::cout<<pfoot_ini[0][2]<<" "<<BodyHeight<<std::endl;

//        if (x_ini_idx<100&&x_ini_idx>0&&BodyHeight-height_map(x_ini_idx,y_ini_idx)<0.25) BodyHeight=height_map(x_ini_idx,y_ini_idx)+0.25;
////        else if(BodyHeight-height_map(x_ini_idx,y_ini_idx)<0.25) BodyHeight=0.29;
//        else if (std::abs(pfoot_ini[0][2]-pfoot_ini[3][2])<=0.02) BodyHeight=0.29+height_map(x_ini_idx,y_ini_idx);
//        else BodyHeight=0.29;
//        if (std::abs(seResult.position[2]-BodyHeight)>0.02)
//            seResult.position[2]=BodyHeight;
//        std::cout<<seResult.position[2]<<" "<<x_ini_idx<<" "<<y_ini_idx<<std::endl;
    }
    // gaitstate
    Vec4<float> contactStates = gait->getContactState();
    Vec4<float> swingStates = gait->getSwingState();
    // some first time initialization
    if(firstRun)
    {
        world_position_desired[0] = seResult.position[0];
        world_position_desired[1] = seResult.position[1];
        world_position_desired[2] = seResult.rpy[2];

        for(int i = 0; i < 4; i++)
        {
            footSwingTrajectories[i].setHeight(StepHeight);
            footSwingTrajectories[i].setInitialPosition(pFoot[i]);
            footSwingTrajectories[i].setFinalPosition(pFoot[i]);
            //1111
            pfoot_ini[i] = pFoot[i];
            pfoot_log[i] = pFoot[i];
            pDesFootWorld[i]=pFoot[i];
            if (swingStates[i]>=0)
            {
                pfoot_des[i] = pfoot_log[i]+delta;
            }
            else if(contactStates[i]>=0)
            {
                pfoot_des[i]=pfoot_log[i];
            }
            float side_sign[4] = {-1, 1, -1, 1};
            Vec3<float> offset(0, side_sign[i] * .065, 0);
            _UpdateFootholdNew(pfoot_des[i], pfoot_ini[i], seResult.position, height_map, idx_map);
        }
        pzFootMin = pfoot_des[0][2];
        firstRun = false;
    }

    // foot placement
    swingTimes[0] = dtMPC * gait->_swing;
    swingTimes[1] = dtMPC * gait->_swing;
    swingTimes[2] = dtMPC * gait->_swing;
    swingTimes[3] = dtMPC * gait->_swing;


    for(int i = 0; i < 4; i++) {

        if(firstSwing[i]) {
            swingTimeRemaining[i] = swingTimes[i];
        } else {
            swingTimeRemaining[i] -= dt;
        }

        float p_rel_max = 0.3f;

        // Using the estimated velocity is correct
        float pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
                        .03f*(seResult.vWorld[0]-v_des_world[0]) +
                        (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*v_rpy_des[2]);

        float pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
                        .03f*(seResult.vWorld[1]-v_des_world[1]) +
                        (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*v_rpy_des[2]);

        pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

        _fin_foot_loc[i] = pfoot_des[i];


    }
    // calc gait
    gait->setIterations(iterationsBetweenMPC, iterationCounter);
    iterationCounter++;

    // load LCM leg swing gains
    Kp << 700, 0, 0,
            0, 700, 0,
            0, 0, 150;
    Kp_stance = 0*Kp;

    Kd << 11, 0, 0,
            0, 11, 0,
            0, 0, 11;
    Kd_stance = Kd;

    int* mpcTable = gait->mpc_gait();
    updateMPCIfNeeded(mpcTable, data);
    footsteps next_steps;
    Vec4<float> se_contactState(0,0,0,0);
    for(int foot = 0; foot < 4; foot++)
    {
        float contactState = contactStates[foot];
        float swingState = swingStates[foot];

        if(swingState > 0) // foot is in swing
        {
            if(firstSwing[foot])
            {
                firstSwing[foot] = false;
//        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
//        pfoot_ini[foot]=pFoot[foot];
            }
            footSwingTrajectories[foot].setFinalPosition(pfoot_des[foot]);
            footSwingTrajectories[foot].setInitialPosition(pfoot_ini[foot]);
//        std::cout<<foot<<" ini "<<pfoot_ini[foot][0]<<" des "<<pfoot_des[foot][0]<<" "<<swingState<<std::endl;
//      footSwingTrajectories[foot].setHeight(_fin_foot_loc[foot][2]+0.05);
            footSwingTrajectories[foot].setHeight(StepHeight);
//std::cout<<foot <<" "<<Pf[foot][1]<<std::endl;
            footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

            pDesFootWorld[foot] = footSwingTrajectories[foot].getPosition();
            Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
            Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld[foot] - seResult.position)
                                  - data._quadruped->getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

            // Update for WBC
            pFoot_des[foot] = pDesFootWorld[foot];
            vFoot_des[foot] = vDesFootWorld;
            aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

            if(!data.userParameters->use_wbc){
                data._legController->commands[foot].pDes = pDesLeg;
                data._legController->commands[foot].vDes = vDesLeg;
                data._legController->commands[foot].kpCartesian = Kp;
                data._legController->commands[foot].kdCartesian = Kd;

                //singularity barrier
                data._legController->commands[foot].tauFeedForward[2] =
                        50*(data._legController->datas[foot].q(2)<.1)*data._legController->datas[foot].q(2);
            }
        }
        else // foot is in stance
        {
            if (firstSwing[foot] == false) {

                firstSwing[foot] = true;
                pfoot_ini[foot]=pFoot[foot];
                //  update body height
                if (pfoot_des[foot][2] > pzFootMin && pfoot_des[foot][2] - pzFootMin > 0.01) {
                    pzFootMin = pfoot_des[foot][2];
                    BodyHeight = pzFootMin + 0.27;
                }
                //  update body pitch
                double tan_p = 0.0;
                if (foot == 1){
                    tan_p = (pFoot[foot][2] - pFoot[2][2])/(pFoot[foot][0] - pFoot[2][0]);
                }else if (foot ==0){
                    tan_p = (pFoot[foot][2] - pFoot[3][2])/(pFoot[foot][0] - pFoot[3][0]);
                }
                if (std::abs(tan_p) > 0.10)  rpy_comp[1] += -atan(tan_p);

                //  set the max steps
                if (stepcount[2] >= sumStep) {
//                pfoot_des[foot]=pDesFootWorld[foot];
//                pfoot_des[foot][2] = 0.18;
//                pfoot_des[foot]=pfoot_log[foot];
                    vx = 0.0;
//                StepHeight=0.;
//                _UpdateFootholdNew(pfoot_des[foot], seResult.position, height_map, idx_map);
                }
                else {
//           pfoot_des[foot] = pDesFootWorld[foot] + delta;
                    pfoot_log[foot] = pfoot_log[foot] + delta;//next step's nominal xy location
                    pfoot_des[foot] = pfoot_log[foot];
                    _UpdateFootholdNew(pfoot_des[foot], pfoot_ini[foot], seResult.position, height_map, idx_map);

//           pfoot_des[foot][2] = z_list[stepcount[foot]][foot];
                }
            }
            if (contactState<0.01&&contactState>0) {
                stepcount[foot]++;
                std::cout<<foot<<" ini-x "<<pfoot_ini[foot][0]<<" nom-x "<<pfoot_log[foot][0]<<" des-x "<<pfoot_des[foot][0];
                std::cout<<" ini-z "<<pfoot_ini[foot][2]<<" des-z "<<pfoot_des[foot][2]<<" step"<<stepcount[foot] <<" pitch "<<rpy_comp[1]<<std::endl;
                if (!checkStep_OK)  std::cout<<"no proper step found!"<<std::endl;
            }
//      std::cout<<foot<<" "<<pfoot_ini[foot][1]<<" "<<pfoot_des[foot][1]<<std::endl;
            Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
            Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld[foot] - seResult.position) - data._quadruped->getHipLocation(foot);
            Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
            //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

            if(!data.userParameters->use_wbc){
                data._legController->commands[foot].pDes = pDesLeg;
                data._legController->commands[foot].vDes = vDesLeg;
                data._legController->commands[foot].kpCartesian = Kp_stance;
                data._legController->commands[foot].kdCartesian = Kd_stance;

                data._legController->commands[foot].forceFeedForward = f_ff[foot];
                data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
            }
            se_contactState[foot] = contactState;
        }
        next_steps.xpos[foot]=pfoot_des[foot][0];
        next_steps.ypos[foot]=pfoot_des[foot][1];
        next_steps.zpos[foot]=pfoot_des[foot][2];
    }
    footLCM.publish("footsteps", &next_steps);
    // se->set_contact_state(se_contactState); todo removed
    data._stateEstimator->setContactPhase(se_contactState);

    // Update For WBC
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = BodyHeight;//_body_height;

    vBody_des[0] = vx;//v_des_world[0];
    vBody_des[1] = vy;//v_des_world[1];
//  if (BodyHeight-pfoot_des[2][2]>0.02) vBody_des[2] = 0.1;
//  else
    vBody_des[2] = 0.0;

    pBody_RPY_des[0] = 0.;
    pBody_RPY_des[1] = rpy_comp[1];
    pBody_RPY_des[2] = rpy_des[2];

    vBody_Ori_des[0] = 0.;
    vBody_Ori_des[1] = 0.;
    vBody_Ori_des[2] = v_rpy_des[2];

    //contact_state = gait->getContactState();
    contact_state = gait->getContactState();
    // END of WBC Update
}

void VisionMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) {
    //iterationsBetweenMPC = 30;
    if((iterationCounter % iterationsBetweenMPC) == 0)
    {
        auto seResult = data._stateEstimator->getResult();
        float* p = seResult.position.data();

        if(current_gait == 4)    {
            float trajInitial[12] = {(float)rpy_des[0], // Roll
                                     (float)0.0, // Pitch
                                     (float)stand_traj[5],
                                     (float)stand_traj[0],
                                     (float)stand_traj[1],
                                     (float)BodyHeight,//_body_height,
                                     0,0,0,0,0,0};

            for(int i = 0; i < horizonLength; i++)
                for(int j = 0; j < 12; j++)
                    trajAll[12*i+j] = trajInitial[j];
        } else {
            const float max_pos_error = .1;
            float xStart = world_position_desired[0];
            float yStart = world_position_desired[1];

            if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
            if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

            if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
            if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

            world_position_desired[0] = xStart;
            world_position_desired[1] = yStart;

            float trajInitial[12] = {(float)rpy_comp[0],  // 0
                                     (float)rpy_comp[1],    // 1
                                     (float)rpy_des[2],    // 2
                                     xStart,                                   // 3
                                     yStart,                                   // 4
                                     (float)BodyHeight,      // 5
                                     0,                                        // 6
                                     0,                                        // 7
                                     (float)v_rpy_des[2],  // 8
                                     vx,//v_des_world[0],                           // 9
                                     v_des_world[1],                           // 10
                                     0};                                       // 11

            for(int i = 0; i < horizonLength; i++) {
                for(int j = 0; j < 12; j++)  trajAll[12*i+j] = trajInitial[j];

                if(i == 0) // start at current position
                {
                    trajAll[2] = seResult.rpy[2];
                } else {
                    trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * vx;//v_des_world[0];
                    trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];

                    trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * v_rpy_des[2];
                }
            }
        }
        solveDenseMPC(mpcTable, data);
    }
}

void VisionMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
    auto seResult = data._stateEstimator->getResult();

    float Q[12] = {0.25, 0.25, 10, 2, 20, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
    //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
    float yaw = seResult.rpy[2];
    float* weights = Q;
    float alpha = 4e-5; // make setting eventually
    float* p = seResult.position.data();
    float* v = seResult.vWorld.data();
    float* w = seResult.omegaWorld.data();
    float* q = seResult.orientation.data();

    float r[12];
    for(int i = 0; i < 12; i++) r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

    if(alpha > 1e-4) {
        std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
        alpha = 1e-5;
    }

    dtMPC = dt * iterationsBetweenMPC;
    vision_setup_problem(dtMPC,horizonLength,0.4,120);
    vision_update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);

    for(int leg = 0; leg < 4; leg++)
    {
        Vec3<float> f;
        for(int axis = 0; axis < 3; axis++)
            f[axis] = vision_get_solution(leg*3 + axis);

        f_ff[leg] = -seResult.rBody * f;
        // Update for WBC
        Fr_des[leg] = f;
    }
}

