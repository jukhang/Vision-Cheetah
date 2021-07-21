/*============================ Vision =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Vision.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <fstream>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Vision<T>::FSM_State_Vision(
        ControlFSMData<T>* _controlFSMData)
        : FSM_State<T>(_controlFSMData, FSM_StateName::VISION, "VISION"),
          vision_MPC(_controlFSMData->controlParameters->controller_dt,
                     30 / (1000. * _controlFSMData->controlParameters->controller_dt),
                     _controlFSMData->userParameters),
          cMPCOld(_controlFSMData->controlParameters->controller_dt,
                  30 / (1000. * _controlFSMData->controlParameters->controller_dt),
                  _controlFSMData->userParameters),
          _visionLCM(getLcmUrl(255))
{
    // Set the safety checks
    this->turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;

    // Initialize GRF and footstep locations to 0s
    this->footFeedForwardForces = Mat34<T>::Zero();
    this->footstepLocations = Mat34<T>::Zero();
    _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
    _wbc_data = new LocomotionCtrlData<T>();
    zero_vec3.setZero();
    _global_robot_loc.setZero();
    _robot_rpy.setZero();

//1012
    _visionLCM.subscribe("heightmapnew", &FSM_State_Vision<T>::handleHeightmapnewLCM, this);
//    _visionLCM.subscribe("heightmap333", &FSM_State_Vision<T>::handleHeightmap333LCM, this);
//1030
    _visionLCM.subscribe("traversability_float", &FSM_State_Vision<T>::handleIndexmapfloatLCM, this);

//    _visionLCM.subscribe("local_heightmap", &FSM_State_Vision<T>::handleHeightmapLCM, this);
//    _visionLCM.subscribe("traversability", &FSM_State_Vision<T>::handleIndexmapLCM, this);
    _visionLCM.subscribe("global_to_robot", &FSM_State_Vision<T>::handleLocalization, this);
    _visionLCMThread = std::thread(&FSM_State_Vision<T>::visionLCMThread, this);
    _height_map = DMat<T>::Zero(x_size, y_size);
    _height_map333 = DMat<T>::Zero(x333_size, y333_size);
    _height_mapnew = DMat<T>::Zero(xnew_size, ynew_size);
    _idx_map = DMat<int>::Zero(x_size, y_size);
    idx_map = DMat<float>::Zero(x_size, y_size);

}
template<typename T>
void FSM_State_Vision<T>::handleLocalization(const lcm::ReceiveBuffer* rbuf,
                                             const std::string& chan, const localization_lcmt* msg){
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<3; ++i){
        _robot_rpy[i] = msg->rpy[i];
        _global_robot_loc[i] = msg->xyz[i];
    }
    _b_localization_data = true;
}

template<typename T>
void FSM_State_Vision<T>::handleHeightmapLCM(const lcm::ReceiveBuffer *rbuf,
                                             const std::string &chan,
                                             const heightmap_t *msg) {
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<x_size; ++i){
        for(size_t j(0); j<y_size; ++j){
            _height_map(i,j) = msg->map[i][j];
        }
    }
}

template<typename T>
void FSM_State_Vision<T>::handleHeightmap333LCM(const lcm::ReceiveBuffer *rbuf,
                                                const std::string &chan,
                                                const heightmap333_t *msg) {
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<x333_size; ++i){
        for(size_t j(0); j<y333_size; ++j){
            _height_map333(i,j) = msg->map[i][j];
        }
    }
}
template<typename T>
void FSM_State_Vision<T>::handleHeightmapnewLCM(const lcm::ReceiveBuffer *rbuf,
                                                const std::string &chan,
                                                const heightnew_t *msg) {
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<xnew_size; ++i){
        for(size_t j(0); j<ynew_size; ++j){
            _height_mapnew(i,j) = msg->map[i][j];
        }
    }
}
template<typename T>
void FSM_State_Vision<T>::handleIndexmapLCM(const lcm::ReceiveBuffer *rbuf,
                                            const std::string &chan,
                                            const traversability_map_t *msg) {
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<x_size; ++i){
        for(size_t j(0); j<y_size; ++j){
            _idx_map(i,j) = msg->map[i][j];
        }
    }
}
template<typename T>
void FSM_State_Vision<T>::handleIndexmapfloatLCM(const lcm::ReceiveBuffer *rbuf,
                                            const std::string &chan,
                                            const traversability_float_t *msg) {
    (void)rbuf;
    (void)chan;

    for(size_t i(0); i<x_size; ++i){
        for(size_t j(0); j<y_size; ++j){
            idx_map(i,j) = msg->map[i][j];
        }
    }
}

template <typename T>
void FSM_State_Vision<T>::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();
    vision_MPC.initialize();

    if(_b_localization_data){
        _updateStateEstimator();
        _ini_body_pos = _global_robot_loc;
        _ini_body_ori_rpy = _robot_rpy;
    }else{
        _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;
        _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
    }
    printf("[FSM VISION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Vision<T>::run() {
    if(_b_localization_data){
        _updateStateEstimator();
    }
    // Call the locomotion control logic for this iteration
    Vec3<T> des_vel; // x,y, yaw

//    _UpdateObstacle();
//    _UpdateObstacle_new();
//    _UpdateObstacle_trav();
    // Vision Walking
    _UpdateVelCommand(des_vel);
    _LocomotionControlStep(des_vel);

    // Convex Locomotion
    //_RCLocomotionControl();

    // Stand still
    //_JPosStand();
//    _Visualization(des_vel);
}

template <typename T>
void FSM_State_Vision<T>::_RCLocomotionControl() {
    cMPCOld.run<T>(*this->_data);

    if(this->_data->userParameters->use_wbc > 0.9){
        _wbc_data->pBody_des = cMPCOld.pBody_des;
        _wbc_data->vBody_des = cMPCOld.vBody_des;
        _wbc_data->aBody_des = cMPCOld.aBody_des;

        _wbc_data->pBody_RPY_des = cMPCOld.pBody_RPY_des;
        _wbc_data->vBody_Ori_des = cMPCOld.vBody_Ori_des;

        for(size_t i(0); i<4; ++i){
            _wbc_data->pFoot_des[i] = cMPCOld.pFoot_des[i];
            _wbc_data->vFoot_des[i] = cMPCOld.vFoot_des[i];
            _wbc_data->aFoot_des[i] = cMPCOld.aFoot_des[i];
            _wbc_data->Fr_des[i] = cMPCOld.Fr_des[i];
        }
        _wbc_data->contact_state = cMPCOld.contact_state;

        _wbc_ctrl->run(_wbc_data, *this->_data);

    }
}

template<typename T>
void FSM_State_Vision<T>::_updateStateEstimator(){
    StateEstimate<T> * estimate_handle = this->_data->_stateEstimator->getResultHandle();

    //printf("global robot:%f,%f,%f\n", _global_robot_loc[0], _global_robot_loc[1], _global_robot_loc[2]);
    estimate_handle->position = _global_robot_loc;
    estimate_handle->rpy = _robot_rpy;
    //
    estimate_handle->rBody = rpyToRotMat(_robot_rpy);
    estimate_handle->orientation = rpyToQuat(_robot_rpy);
}

template<typename T>
void FSM_State_Vision<T>::_JPosStand(){
    Vec3<T> stand_jpos;
    stand_jpos << 0.f, -.8f, 1.6f;
    for(int leg(0); leg<4;++leg){
        this->jointPDControl(leg, stand_jpos, zero_vec3);
    }
}
template <typename T>
void FSM_State_Vision<T>::_UpdateObstacle_new() {

    _obs_list.clear();
    T obstacle_height = 0.15;
    T threshold_gap = 0.1;
    T cam_offset = 0.2;
    bool add_obs(true);
    Vec3<T> robot_loc;
//    std::cout<<_global_robot_loc<<endl;
    if (_b_localization_data) { // 由于没有定位数据，false
        robot_loc = _global_robot_loc;
    } else {
        robot_loc = (this->_data->_stateEstimator->getResult()).position;
    }
    Vec3<T> obs;
    obs[2] = 0.2;
    T dist = 0;
    for (size_t i(0); i < xnew_size; ++i) {//101
        for (size_t j(0); j < ynew_size; ++j) {//101
            if ((_height_mapnew(i, j) > obstacle_height)) { // if too high point
                add_obs = true;
                obs[0] = -(i * grid_size) + 100 * grid_size + robot_loc[0]+cam_offset;//WORLD FRAME OBS LOCATION
                obs[1] = -(j * grid_size) + 50 * grid_size + robot_loc[1];
                // check distance with already added obstacle
                for (size_t idx_obs(0); idx_obs < _obs_list.size(); ++idx_obs) {
                    dist = sqrt((obs[0] - _obs_list[idx_obs][0]) * (obs[0] - _obs_list[idx_obs][0]) +
                                (obs[1] - _obs_list[idx_obs][1]) * (obs[1] - _obs_list[idx_obs][1]));
                    if (dist < threshold_gap) {
                        add_obs = false;
                        break;
                    }
                }// distance check
                if (add_obs) {
                    _obs_list.push_back(obs);
//                    cout<<"x "<<obs[0]<<" y "<<obs[1]<<" i "<<i<<" j "<<j<<endl;
                }
            }
        } // y loop
    } // x loop

}

template <typename T>
void FSM_State_Vision<T>::_UpdateObstacle_trav() {
    traversability_float_t trav_lcm;

    Vec3<T> robot_loc;
//    std::cout<<_global_robot_loc<<endl;
    if (_b_localization_data) { // 由于没有定位数据，false
        robot_loc = _global_robot_loc;
    } else {
        robot_loc = (this->_data->_stateEstimator->getResult()).position;
    }

    float w_sd=60.,w_sl=30.,w_max=10.,w_min=10.;//score wieght

    for (size_t i(0); i < xnew_size; ++i) {//101
        for (size_t j(0); j < ynew_size; ++j) {//101
//            float travmap[100][100];
            if (i>=1&&j>=1&&i+1<xnew_size&&j+1<ynew_size)
            {
                float slope=0, sum=0;
                float hmax=_height_mapnew(i,j), hmin=_height_mapnew(i,j);
                float mean,sd,score;
                float tmpmap[9];
                int k=0;
                for (size_t m(i-1); m<=(i+1);m++){
                    for (size_t n(j-1); n<=(j+1);n++){
                        if (!(m==i && n==j))
//                            cout<<_height_mapnew(m,n)<<endl;
                        // slope
                        slope=abs((_height_mapnew(i, j)-_height_mapnew(m, n))/sqrt((int)((i-m)*(i-m)+(j-n)*(j-n))))+slope;
                        //max
                        if (_height_mapnew(m,n)>hmax)
                            hmax=_height_mapnew(m,n);
                        //min
                        if (_height_mapnew(m,n)<hmin)
                            hmin=_height_mapnew(m,n);
                        sum=sum+_height_mapnew(m,n);
                        tmpmap[k]=_height_mapnew(m,n);
                        k++;
                    }
                }
                //标准差
                mean=sum/9;
                sd=sqrt(((tmpmap[0]-mean)*(tmpmap[0]-mean)+(tmpmap[1]-mean)*(tmpmap[1]-mean)+(tmpmap[2]-mean)*(tmpmap[2]-mean)+(tmpmap[3]-mean)*(tmpmap[3]-mean)+(tmpmap[4]-mean)*(tmpmap[4]-mean)
                        +(tmpmap[5]-mean)*(tmpmap[5]-mean)+(tmpmap[6]-mean)*(tmpmap[6]-mean)+(tmpmap[7]-mean)*(tmpmap[7]-mean)+(tmpmap[8]-mean)*(tmpmap[8]-mean))/9.0);
                //斜率
                slope=slope/8;
                //traversability map
                score = w_sd*sd + w_sl*slope + w_max*(hmax-_height_mapnew(i,j)) + w_min*(_height_mapnew(i,j)-hmin);
                trav_lcm.map[i][j]=score;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << " slope: " << slope <<" sd: "<<sd<<endl;
            }
            else if((i==0 || j==0)&&i+1<xnew_size&&j+1<ynew_size) {
                trav_lcm.map[i][j] = 0;
//                cout << "i" << i << " j" << j << " score " << trav_lcm.map[i][j] << endl;
            }
        } // y loop
    } // x loop

    _visionLCM.publish("traversability_float", &trav_lcm);
}
template <typename T>
void FSM_State_Vision<T>::_UpdateObstacle() {
    _obs_list.clear();

/* // TEST
    Vec3<T> test1,test2,test3;
    test1<< 0.7, 0.20, 0.27;
    test2<< 0.7, -0.2, 0.17;
    test3<< 0.7, -0.4, 0.07;
    _obs_list.push_back(test1);
    _obs_list.push_back(test2);
    _obs_list.push_back(test3);
    */
    Vec3<T> obs_loc333;
    T dist;
    T obstacle_height = 0.15;
    T threshold_gap = 0.08;
    bool add_obs;
    Vec3<T> robot_loc;
    //get robot global location
    if (_b_localization_data) { //TRUE
        robot_loc = _global_robot_loc;
    } else {
        robot_loc = (this->_data->_stateEstimator->getResult()).position;
    }
    //std::cout<<robot_loc[0]<<" "<<robot_loc[1]<<endl;
    //m,n range is 100 (Robot-centric)
    size_t m_low= (robot_loc[0]/grid_map_size)+((x333_size*1-x_size*scale)/2);
    size_t n_low= (robot_loc[1]/grid_map_size)+((y333_size*1-y_size*scale)/2);
    size_t m_high= (robot_loc[0]/grid_map_size)+((x333_size*1+x_size*scale)/2);
    size_t n_high=  (robot_loc[1]/grid_map_size)+((y333_size*1+y_size*scale)/2);
//    std::cout<<m_low<<" "<<m_high<<std::endl;
    for (size_t m(m_low); m <m_high; ++m) {
        for (size_t n(n_low); n <n_high; ++n) {
            if ((0 <=m && m < x333_size && 0 <=n && n < y333_size)){
                //do nothing: The detection range exceeds the map range
//std::cout<<m<<" "<<n<<std::endl;
                if (_height_map333(m, n) > obstacle_height || _height_map333(m, n) < -obstacle_height) {
                    add_obs = true;
                    obs_loc333[0] = ( m - x333_size*0.5) * grid_map_size;//WORLD FRAME OBS LOCATION
                    obs_loc333[1] = ( n - y333_size*0.5) * grid_map_size;
                    if (_height_map333(m, n) < -obstacle_height)
                        obs_loc333[2] = -0.2;
                    else obs_loc333[2] = 0.2;
                    // check distance with already added obstacle
                    for (size_t idx_obs(0); idx_obs < _obs_list.size(); ++idx_obs) {
                        dist = sqrt((obs_loc333[0] - _obs_list[idx_obs][0]) * (obs_loc333[0] - _obs_list[idx_obs][0]) +
                                    (obs_loc333[1] - _obs_list[idx_obs][1]) * (obs_loc333[1] - _obs_list[idx_obs][1]));
                        if (dist < threshold_gap) {
                            add_obs = false;
                            break;
                        }
                    }// add obs 3d location
                    if (add_obs) {
                        _obs_list.push_back(obs_loc333);
//                    std::cout << obs_loc333[0] << " " << obs_loc333[1] << " " << obs_loc333[2] << std::endl;
                    }
                }
            }}
    }
    //}
    /*T obstacle_height = 0.15;
    T threshold_gap = 0.08;
    bool add_obs(true);
    Vec3<T> robot_loc;
    if (_b_localization_data) { //TRUE
        robot_loc = _global_robot_loc;
    } else {
        robot_loc = (this->_data->_stateEstimator->getResult()).position;
    }
    Vec3<T> obs;
    obs[2] = 0.27;
    T dist = 0;
    for (size_t i(0); i < x_size; ++i) {//100
        for (size_t j(0); j < y_size; ++j) {//100
            //if(_height_map333(i,j)!=0){std::cout<<_height_map333(i,j)<<"!!!"<<std::endl;}
            if ((_height_map(i, j) > obstacle_height)) { // if too high point
                add_obs = true;
                obs[0] = i * grid_size - 50 * grid_size + robot_loc[0];//WORLD FRAME OBS LOCATION
                obs[1] = j * grid_size - 50 * grid_size + robot_loc[1];

                // check distance with already added obstacle
                for (size_t idx_obs(0); idx_obs < _obs_list.size(); ++idx_obs) {
                    dist = sqrt((obs[0] - _obs_list[idx_obs][0]) * (obs[0] - _obs_list[idx_obs][0]) +
                                (obs[1] - _obs_list[idx_obs][1]) * (obs[1] - _obs_list[idx_obs][1]));
                    if (dist < threshold_gap) {
                        add_obs = false;
                    }
                }// distance check
                if (add_obs) {
                    _obs_list.push_back(obs);
                }
            }
        } // y loop
    } // x loop*/

//_print_obstacle_list();
}
template <typename T>
void FSM_State_Vision<T>::_print_obstacle_list(){
    for(size_t i(0); i < _obs_list.size(); ++i){
        printf("%lu th obstacle: %f, %f, %f\n", i, _obs_list[i][0], _obs_list[i][1], _obs_list[i][2] );
    }
}

template <typename T>
void FSM_State_Vision<T>::_Visualization(const Vec3<T> & des_vel){
    velocity_visual_t vel_visual;
    for(size_t i(0); i<3; ++i){
        vel_visual.vel_cmd[i] = des_vel[i];
        // 1022
        if (i==2) vel_visual.base_position[2] = (this->_data->_stateEstimator->getResult()).rpy[2];
        else vel_visual.base_position[i] = (this->_data->_stateEstimator->getResult()).position[i];
    }
    _visionLCM.publish("velocity_cmd", &vel_visual);


    _obs_visual_lcm.num_obs = _obs_list.size();
    for(size_t i(0); i<_obs_list.size(); ++i){
        _obs_visual_lcm.location[i][0] = _obs_list[i][0];
        _obs_visual_lcm.location[i][1] = _obs_list[i][1];
        _obs_visual_lcm.location[i][2] = _obs_list[i][2];
    }
    _obs_visual_lcm.sigma = 0.15;
    _obs_visual_lcm.height = 0.5;
    _visionLCM.publish("obstacle_visual", &_obs_visual_lcm);
}

template <typename T>
void FSM_State_Vision<T>::_UpdateVelCommand(Vec3<T> & des_vel) {
    des_vel.setZero();
//
//    Vec3<T> target_pos, curr_pos, curr_ori_rpy;
//
//  target_pos.setZero();
//
//  T moving_time = 25.0;
//  T curr_time = (T)iter * 0.002;
//  //if(curr_time > moving_time){
//    //curr_time = moving_time;
//  //}
//  //target_pos[0] += 2.0 * curr_time/moving_time;
//  //target_pos[0] += 2.5 * curr_time/moving_time;
//  //target_pos[0] = 0.7 * (1-cos(2*M_PI*curr_time/moving_time));
//  target_pos[0] = 1.0 * (1-cos(2*M_PI*curr_time/moving_time));
//
//  if(_b_localization_data){
//    curr_pos = _global_robot_loc;
//    curr_pos -= _ini_body_pos;
//    curr_ori_rpy = _robot_rpy;
//  }else{
//    curr_pos = (this->_data->_stateEstimator->getResult()).position;
//    curr_pos -= _ini_body_pos;
//    curr_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
//
//  }
//  target_pos = rpyToRotMat(_ini_body_ori_rpy).transpose() * target_pos;
//  des_vel[0] = 0.7 * (target_pos[0] - curr_pos[0]);
//  des_vel[1] = 0.7 * (target_pos[1] - curr_pos[1]);
//  des_vel[2] = 0.7 * (_ini_body_ori_rpy[2] - curr_ori_rpy[2]);
//
//  //des_vel[0] = 0.5;
//  T inerproduct;
//  T x, y, x_obs, y_obs;
//  T vel_x, vel_y;
//  T sigma(0.15);
//  T h(0.5);
//  for(size_t i(0); i<_obs_list.size(); ++i){
//    x = curr_pos[0];
//    y = curr_pos[1];
//    x_obs = _obs_list[i][0];
//    y_obs = _obs_list[i][1];
//    h = _obs_list[i][2];
//
//    inerproduct = (x-x_obs)*(x-x_obs) + (y-y_obs)*(y-y_obs);
//    vel_x = h*((x-x_obs)/ sigma/sigma)*exp(-inerproduct/(2*sigma*sigma));
//    vel_y = h*((y-y_obs)/ sigma/sigma)*exp(-inerproduct/(2*sigma*sigma));
//    //printf("vel_x, y: %f, %f\n", vel_x, vel_y);
//
//    des_vel[0] += vel_x;
//    des_vel[1] += vel_y;
//  }
//  //printf("des vel_x, y: %f, %f\n", des_vel[0], des_vel[1]);
//
//  des_vel[0] = fminf(fmaxf(des_vel[0], -1.), 1.);
//  des_vel[1] = fminf(fmaxf(des_vel[1], -1.), 1.);

    if ( this->_data->controlParameters->use_rc ) {
        const rc_control_settings* rc_cmd = this->_data->_desiredStateCommand->rcCommand;
        des_vel[0]= rc_cmd->v_des[0]+0.15;
        des_vel[1] = rc_cmd->v_des[1] ;
        des_vel[2] = (-rc_cmd->omega_des[2]);
    } else {
        des_vel[0] = this->_data->_desiredStateCommand->leftAnalogStick[1]*0.5;
        des_vel[1] = this->_data->_desiredStateCommand->leftAnalogStick[0]*0.2;
        des_vel[2] = this->_data->_desiredStateCommand->rightAnalogStick[0]*0.5;
    }
    des_vel=this->_data->_stateEstimator->getResult().rBody.transpose() * des_vel;


    des_vel[0] = fminf(fmaxf(des_vel[0], -1.), 1.);
    des_vel[1] = fminf(fmaxf(des_vel[1], -1.), 1.);
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Vision<T>::checkTransition() {
    // Get the next state
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode) {
        case K_VISION:
            break;

        case K_LOCOMOTION:
            this->nextStateName = FSM_StateName::LOCOMOTION;
            this->transitionDuration = 0.0;
            break;

        case K_BALANCE_STAND:
            // Requested change to BALANCE_STAND
            this->nextStateName = FSM_StateName::BALANCE_STAND;

            // Transition time is immediate
            this->transitionDuration = 0.0;

            break;

        case K_PASSIVE:
            // Requested change to BALANCE_STAND
            this->nextStateName = FSM_StateName::PASSIVE;

            // Transition time is immediate
            this->transitionDuration = 0.0;

            break;

        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            this->transitionDuration = 0.;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_VISION << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }

    // Return the next state name to the FSM
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Vision<T>::transition() {
    // Switch FSM control mode
    switch (this->nextStateName) {
        case FSM_StateName::BALANCE_STAND:
            _LocomotionControlStep(zero_vec3);

            iter++;
            if (iter >= this->transitionDuration * 1000) {
                this->transitionData.done = true;
            } else {
                this->transitionData.done = false;
            }

            break;

        case FSM_StateName::PASSIVE:
            this->turnOffAllSafetyChecks();
            this->transitionData.done = true;

            break;

        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;

        case FSM_StateName::LOCOMOTION:
            this->transitionData.done = true;
            break;


        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition"
                      << std::endl;
    }

    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Vision<T>::onExit() {
    // Nothing to clean up when exiting
    iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Vision<T>::_LocomotionControlStep(const Vec3<T> & des_vel) {
    // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

    // Contact state logic
//    vision_MPC.run<T>(*this->_data, des_vel, _height_map, _idx_map);
    vision_MPC.run<T>(*this->_data, des_vel, _height_mapnew, idx_map);

    if (this->_data->userParameters->use_wbc > 0.9) {
        _wbc_data->pBody_des = vision_MPC.pBody_des;
        _wbc_data->vBody_des = vision_MPC.vBody_des;
        _wbc_data->aBody_des = vision_MPC.aBody_des;

        _wbc_data->pBody_RPY_des = vision_MPC.pBody_RPY_des;
        _wbc_data->vBody_Ori_des = vision_MPC.vBody_Ori_des;

        for (size_t i(0); i < 4; ++i) {
            _wbc_data->pFoot_des[i] = vision_MPC.pFoot_des[i];
            _wbc_data->vFoot_des[i] = vision_MPC.vFoot_des[i];
            _wbc_data->aFoot_des[i] = vision_MPC.aFoot_des[i];
            _wbc_data->Fr_des[i] = vision_MPC.Fr_des[i];
        }
        _wbc_data->contact_state = vision_MPC.contact_state;
        _wbc_ctrl->run(_wbc_data, *this->_data);

    }
}

template class FSM_State_Vision<float>;