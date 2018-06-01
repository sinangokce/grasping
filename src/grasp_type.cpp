#include <grasping/grasp_type.h>
#include <unistd.h>
#include <stdio.h>



#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

int joint[16];
int stop_table[16];
int desiredisgreater[16];
int velocity = 1;
int sampling_rate = 3000;


float speed_Percentage=1;
float hand_Direction=0;

double desired_position[DOF_JOINTS] = {0.0};
double current_position[DOF_JOINTS] = {0.0};
double initial_position[DOF_JOINTS] = {0.0};

//double dt;
//double tnowsec; 
//double tnowsec_squared; 
//double tstartsec;
//double tstartsec_squared;

bool startclosing = true;
bool first_run;
//bool separated;
//double jointMaxPositions[DOF_JOINTS];
//double jointMinPositions[DOF_JOINTS];


double L1_ind =  0.0164;
double L2_ind =  0.054;
double L3_ind =  0.0384;
double L4_ind =  0.0267;
double Ltx_ind =  0.0;
double Lty_ind =  -0.0435;
double Ltz_ind =  -0.002242;
double angle_ind = DEGREES_TO_RADIANS(5);

double L1x_th =  -0.027;
double L1y_th =  -0.005;
double L1z_th =  0.0399;
double L2_th =  0.0177;
double L3_th =  0.0514;
double L4_th =  0.0423;



Eigen::Matrix<long double, 4, 1> pos;
Eigen::Matrix<long double, 4, 1> pos2;
Eigen::Matrix<long double, 3, 1> pos_ind;
Eigen::Matrix<long double, 3, 1> pos_ind_middle;
Eigen::Matrix<long double, 3, 1> pos_th;
Eigen::Matrix<long double, 3, 1> pos_th_middle;
Eigen::Matrix<long double, 3, 1> pos_middle_th;
Eigen::Matrix<long double, 4, 4> trans_ind;
Eigen::Matrix<long double, 4, 4> trans_ind_tot;


Eigen::Quaterniond q;
Eigen::Quaterniond q1;
Eigen::Matrix3d quat_to_rot;
Eigen::Matrix3d quat1_to_rot;
Eigen::Matrix<long double, 4, 4> trans_th;
Eigen::Matrix<long double, 4, 4> trans_th_tot;
Eigen::Matrix<long double, 4, 4> R1;
Eigen::Matrix<long double, 4, 4> R2;
Eigen::Matrix<long double, 4, 4> R1_ind;
Eigen::Matrix<long double, 3, 1> difference;
double d_0;
double eff_region=0.04;
Eigen::Matrix<long double, 3, 1> lateral_th;
Eigen::Matrix<long double, 3, 1> v_rep;
Eigen::Matrix<long double, 3, 1> v_attr;
Eigen::Matrix<long double, 3, 1> v_tot;
Eigen::Matrix<long double, 3, 1> new_position;


Eigen::Matrix<long double, 3, 3> J_th;
Eigen::Matrix<long double, 3, 3> J_th_inv;

Eigen::Matrix<long double, 3, 3> J;
Eigen::Matrix<long double, 3, 3> J_inv;



//Eigen::Matrix4d Trans_matrix;
//Eigen::Matrix4d R1;

using namespace Eigen;

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-16}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInvTransposed(mat.cols(), mat.rows());
    singularValuesInvTransposed.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInvTransposed(0, i) = Scalar{1} / singularValues(i);
            singularValuesInvTransposed(1, i) = Scalar{1} / singularValues(i);
            singularValuesInvTransposed(2, i) = Scalar{1} / singularValues(i);
        }
    }
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    return (V.cwiseProduct(singularValuesInvTransposed)) * U.transpose();
}


template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse2(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-16}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInvTransposed(mat.cols(), mat.rows());
    singularValuesInvTransposed.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInvTransposed(0, i) = Scalar{1} / singularValues(i);
            singularValuesInvTransposed(1, i) = Scalar{1} / singularValues(i);
        }
    }
    auto U = svd.matrixU();
    auto V = svd.matrixV();
    return (V.cwiseProduct(singularValuesInvTransposed)) * U.transpose();
}


AllegroNodeGraspController::AllegroNodeGraspController() {
         
  initControllerxx();

  grasp_type_sub = nh.subscribe("allegroHand_0/libsss_cmd", 1, &AllegroNodeGraspController::graspTypeControllerCallback, this);

  SpeedPer_sub = nh.subscribe("/lwr/speedPercentage", 10, &AllegroNodeGraspController::speedPerCallback, this);

  desired_state_pub = nh.advertise<sensor_msgs::JointState>("allegroHand_0/joint_cmd", 1);

  sensor_data_sub = nh.subscribe("/LasaDataStream", 10, &AllegroNodeGraspController::sensorDataCallback, this);

  sensor_data_pub = nh.advertise<grasping::sensor_data>(SENSOR_DATA_TOPIC, 10); //plot

  acc_sub = nh.subscribe("allegroHand_0/stop_topic", 1, &AllegroNodeGraspController::accCallback, this);

  //next_state_sub = nh.subscribe(NEXT_STATE_TOPIC, 1, &AllegroNodeGraspController::nextStateCallback, this);

  //current_state_pub = nh.advertise<sensor_msgs::JointState>(CURRENT_LISTENER_TOPIC, 1);

  //stop_pub = nh.advertise<std_msgs::String>(STOP_TOPIC, 1); 

  //stop_table_pub = nh.advertise<grasping::stop_table>(STOP_TABLE_TOPIC, 10); 

  //wentback_pub = nh.advertise<std_msgs::String>("allegroHand_0/libsss_cmd", 1);
}

void AllegroNodeGraspController::speedPerCallback(const handtracker::spper &msg) {
  speed_Percentage = msg.sPer;
  hand_Direction = msg.dir;
}

void AllegroNodeGraspController::accCallback(const std_msgs::String::ConstPtr &msg) {

  const std::string acc_message = msg->data;
  if(acc_message.compare("Acc2") == 0){
    velocity = 2;
  }

  if(acc_message.compare("Acc5") == 0)
    velocity = 5;

  //ROS_INFO("vel = %d", velocity);
}

void AllegroNodeGraspController::sensorDataCallback(const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg) {

  for(int i = 0; i < 16; i++){
    stop_table[i] = 0;
  }  

  float sensor_data[8];
  float *p1;
  p1 = sensor_data;

  mapping(p1, msg);
  plotSensorData(sensor_data);

  int sensor_stop[8];
  int *p2;
  p2 = stop_table;

  for(int i = 0; i < 8; i++){     //THRESHOLD
    if(sensor_data[i] > 1) 
      sensor_stop[i] = 1;
    else  
      sensor_stop[i] = 0;
  }  

  fillStopTable(p2, sensor_stop);

}

void AllegroNodeGraspController::mapping(float *p, const glove_tekscan_ros_wrapper::LasaDataStreamWrapper &msg) {

//THUMB
  *p     = average((msg.ring2_f[10] + msg.ring2_f[11] + msg.pinky2_f[8] + msg.ring2_s[9] + msg.ring2_s[10] + msg.ring2_s[11]),  6);
  *(p+1) = average((msg.ring3_f[4]+msg.ring3_f[5]+msg.ring3_f[6] + msg.index1_s[9]+msg.index1_s[10]+msg.index1_s[11] + msg.middle3_s[7] + msg.ring3_s[0]+msg.ring3_s[1]+msg.ring3_s[2]+msg.ring3_s[4]+msg.ring3_s[5]), 12);

//INDEX
  *(p+2) = average((msg.middle1_f[0]+msg.middle1_f[1]+msg.middle1_f[2]+msg.middle1_f[3]+msg.middle1_f[4]+msg.middle1_f[5]+msg.middle1_f[6]+msg.index1_s[7]+msg.middle1_s[0]+msg.middle1_s[1]+msg.middle1_s[2]+msg.middle1_s[4]+msg.middle1_s[5]), 13);
  *(p+3) = average((msg.index2_f[0]+msg.index2_f[1]+msg.index2_f[2]+msg.index2_f[3]+msg.index2_s[0]+msg.index2_s[1]+msg.index2_s[2]), 7);

//MIDDLE
  *(p+4) = average((msg.ring1_f[0]+msg.ring1_f[1]+msg.ring1_f[2]+msg.ring1_f[3]+msg.ring1_f[4]+msg.ring1_f[5]+msg.ring1_f[6]+msg.ring1_s[0]+msg.ring1_s[1]+msg.ring1_s[2]+msg.ring1_s[4]+msg.ring1_s[5]), 12);
  *(p+5) = average((msg.middle1_f[14]+msg.middle1_f[15]+msg.middle2_f[0]+msg.middle2_f[1]+msg.middle2_f[2]+msg.middle2_f[3]+msg.middle1_s[10]+msg.middle1_s[11]+msg.middle1_s[13]+msg.middle1_s[14]+msg.middle1_s[15]+msg.middle2_s[0]+msg.middle2_s[1]+msg.middle2_s[2]), 14);

//RING
  *(p+6) = average((msg.ring1_f[14]+msg.ring1_f[15]+msg.pinky1_f[0]+msg.pinky1_f[1]+msg.pinky1_f[2]+msg.pinky1_f[3]+msg.pinky1_f[4]+msg.pinky1_f[5]+msg.pinky1_f[6]+msg.pinky1_f[12]+msg.ring1_s[7]+msg.ring1_s[10]+msg.ring1_s[11]+msg.ring1_s[13]+msg.ring1_s[14]+msg.ring1_s[15]+msg.pinky1_s[0]+msg.pinky1_s[1]+msg.pinky1_s[2]+msg.pinky1_s[4]+msg.pinky1_s[5]+msg.pinky1_s[8]), 22);
  *(p+7) = average((msg.ring2_f[0]+msg.ring2_f[1]+msg.ring2_f[2]+msg.ring2_f[3]+msg.ring2_s[0]+msg.ring2_s[1]+msg.ring2_s[2]+msg.ring2_s[3]), 8);
}

void AllegroNodeGraspController::plotSensorData(float sensor_data[]) {

  grasping::sensor_data sensor_data_msg;
  
  sensor_data_msg.upper_thumb  =  sensor_data[0];
  sensor_data_msg.lower_thumb  =  sensor_data[1];
  sensor_data_msg.upper_index  =  sensor_data[2];
  sensor_data_msg.lower_index  =  sensor_data[3];
  sensor_data_msg.upper_middle =  sensor_data[4];
  sensor_data_msg.lower_middle =  sensor_data[5];
  sensor_data_msg.upper_little =  sensor_data[6];
  sensor_data_msg.lower_little =  sensor_data[7];

  sensor_data_pub.publish(sensor_data_msg);

}

void AllegroNodeGraspController::fillStopTable(int *p2, int sensor_stop[]) {

  if(sensor_stop[0] == 1) {
    *(p2+15) = 1;
  }
  else if(sensor_stop[0] == 0) {
    *(p2+15) = 0;
  }

  if(sensor_stop[1] == 1) {
    *(p2+14) = 1;
    *(p2+13) = 1;
    *(p2+12) = 1;
  }
  else if(sensor_stop[1] == 0) {
    *(p2+14) = 0;
    *(p2+13) = 0;
    *(p2+12) = 0;
  }

  if(sensor_stop[2] == 1) {
    *(p2+3) = 1;
  }
  else if(sensor_stop[2] == 0) {
    *(p2+3) = 0;
  }

  if(sensor_stop[3] == 1) {
    *(p2+2) = 1;
    *(p2+1) = 1;
    *(p2+0) = 1;
  }
  else if(sensor_stop[3] == 0) {
    *(p2+2) = 0;
    *(p2+1) = 0;
    *(p2+0) = 0;
  }


  if(sensor_stop[4] == 1) {
    *(p2+7) = 1;
  }
  else if(sensor_stop[4] == 0) {
    *(p2+7) = 0;
  }

  if(sensor_stop[5] == 1) {
    *(p2+6) = 1;
    *(p2+5) = 1;
    *(p2+4) = 1;
  }
  else if(sensor_stop[5] == 0) {
    *(p2+6) = 0;
    *(p2+5) = 0;
    *(p2+4) = 0;
  }

  if(sensor_stop[6] == 1) {
    *(p2+11) = 1;
  }
  else if(sensor_stop[6] == 0){
    *(p2+11)= 0;
  }

  if(sensor_stop[7] == 1) {
    *(p2+10) = 1;
    *(p2+9) = 1;
    *(p2+8) = 1;
  }
  else if(sensor_stop[7] == 0){
    *(p2+10) = 0;
    *(p2+9) = 0;
    *(p2+8) = 0;
  }

}

void AllegroNodeGraspController::graspTypeControllerCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("CTRL: Heard: [%s]", msg->data.c_str());
  const std::string grasp_type = msg->data;

  /*if (first_run)
    for (int i = 0; i < DOF_JOINTS; ++i)
    {
      initial_position[i] = 0;
    }
  else   */                 
    for (int i = 0; i < DOF_JOINTS; ++i)
    {
      initial_position[i] = desired_position[i];
    } 
  
  compareString(grasp_type);

  /*Eigen::Matrix<long double, 3, 1> possss_th;
  possss_th << 0.0482, -0.0395, 0.0947;
  getJointAngles(possss_th);*/


  for (int i = 0; i < DOF_JOINTS; i++) {
    joint[i] = 0;
    stop_table[i] = 0;
  }

  if(first_run)
    //moveToDesiredGraspType();
    moveToDesiredGraspType2();
  else if(!first_run) {
    //potentialField();
    //openHand();
    //separateFingers();//to avoid collisions
    
    //separateSmartly();
    //moveToDesiredGraspType();
    moveToDesiredGraspType2();//not smooth controlling
  }

  first_run = false;
}

void AllegroNodeGraspController::compareString(std::string const &grasp_type) {
  std_msgs::String stop_msg;
  std::stringstream stop_ss;

  for(int i = 0; i < DOF_JOINTS; i++)
    desiredisgreater[i] = 0;

  if (grasp_type.compare("power") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = power[i];
  }

  else if (grasp_type.compare("thumb") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = thumb[i];
  }

  else if (grasp_type.compare("pinch") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = pinch[i];
  }

  else if (grasp_type.compare("lateral") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++) 
      desired_position[i] = lateral[i]; 
  }

  else if (grasp_type.compare("home") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = home_pose[i]; 
  }
}

void AllegroNodeGraspController::getCurrentPosition(double jointAngles[16]) {
  double teta4_th =  jointAngles[15];
  double teta3_th =  jointAngles[14];
  double teta2_th =  -jointAngles[13];
  double teta1_th =  jointAngles[12];

  double teta4_ind =  jointAngles[3];
  double teta3_ind =  jointAngles[2];
  double teta2_ind =  jointAngles[1];
  double teta1_ind =  jointAngles[0];

  /*trans_ind << 1, 0, 0, Ltx_ind, 
             0, cos(angle_ind), -sin(angle_ind), Lty_ind,
             0, sin(angle_ind),  cos(angle_ind), Ltz_ind,
             0, 0, 0, 1;*/ 

  
             
   
  R1 << 1, 0, 0, 0,
        0, cos(teta1_th), -sin(teta1_th), 0,
        0, sin(teta1_th), cos(teta1_th), 0,
        0,0,0,1;

  R1_ind << cos(current_state.position[0]), -sin(current_state.position[0]), 0, 0,
            sin(current_state.position[0]), cos(current_state.position[0]), 0, 0,
            0, 0,  1, 0,
            0,0,0,1;      

  trans_th_tot = trans_th*R1; 
  trans_ind_tot = trans_ind*R1_ind;


  /*pos << cos(teta1_ind)*(L3_ind*sin(teta2_ind+teta3_ind) + L2_ind*sin(teta2_ind) + L4_ind*sin(teta2_ind+teta3_ind+teta4_ind)),
         sin(teta1_ind)*(L3_ind*sin(teta2_ind+teta3_ind) + L2_ind*sin(teta2_ind) + L4_ind*sin(teta2_ind+teta3_ind+teta4_ind)),
         L1_ind + (L3_ind*cos(teta2_ind+teta3_ind) + L2_ind*cos(teta2_ind) + L4_ind*cos(teta2_ind+teta3_ind+teta4_ind)),
         1;*/

  pos << (L3_ind*sin(teta2_ind+teta3_ind) + L2_ind*sin(teta2_ind) + L4_ind*sin(teta2_ind+teta3_ind+teta4_ind)),
         0,
         L1_ind + (L3_ind*cos(teta2_ind+teta3_ind) + L2_ind*cos(teta2_ind) + L4_ind*cos(teta2_ind+teta3_ind+teta4_ind)),
         1;       

  pos_ind << pos(0,0), pos(1,0), pos(2,0);

  pos = trans_ind_tot*pos;

  pos_ind_middle << pos(0,0), pos(1,0), pos(2,0); 

  //std::cout << "pos_ind\n   " << pos_ind << std::endl; 
  //std::cout << "pos_ind_middle2132131\n   " << pos_ind_middle << std::endl; 


  pos <<  L1x_th + cos(teta2_th)*(L4_th*sin(teta3_th+teta4_th) + L3_th*sin(teta3_th)),
          L1y_th + sin(teta2_th)*(L4_th*sin(teta3_th+teta4_th) + L3_th*sin(teta3_th)),
          L2_th + L1z_th + L4_th*cos(teta3_th+teta4_th) + L3_th*cos(teta3_th),
          1;

  pos_th << pos(0,0), pos(1,0), pos(2,0);

  pos = trans_th_tot*pos;           

  pos_th_middle << pos(0,0), pos(1,0), pos(2,0); 

  //std::cout << "pos_ind\n   " << pos_ind << std::endl;  
  //std::cout << "pos_th\n   " << pos_th_middle << std::endl;

  
}

void AllegroNodeGraspController::potentialField() {

  lateral_th << 0.0535676, -0.0423956, -0.0511716;//middle reference
  double Pi = 2*asin(1);

  //getCurrentPosition();

  //std::cout << "pos_th_middle \n " << pos_th_middle << std::endl;
  //std::cout << "pos_ind_middle \n " << pos_ind_middle << std::endl;

  difference << (pos_th_middle(0,0)-pos_ind_middle(0,0)),
                (pos_th_middle(1,0)-pos_ind_middle(1,0)),
                (pos_th_middle(2,0)-pos_ind_middle(2,0));

  d_0 = difference.norm();



  //std::cout << "d_0 = " << d_0 << std::endl;
  double rep_angle; 

  if(difference(0,0) >= 0) 
    rep_angle = atan(difference(1,0)/difference(0,0));
  else 
    rep_angle = Pi + atan(difference(1,0)/difference(0,0));


  //std::cout << "rep_angle \n " << rep_angle << std::endl;

  v_attr << (lateral_th(0,0)-pos_th_middle(0,0)),
            (lateral_th(1,0)-pos_th_middle(1,0)),
            (lateral_th(2,0)-pos_th_middle(2,0));   

  if (d_0 > eff_region) 
    v_rep << 0,0,0;          
  else {
    v_rep << cos(rep_angle)/d_0, sin(rep_angle)/d_0, 1/d_0;
  } 

  v_tot = v_rep+v_attr;

  new_position = pos_th_middle + v_tot;

  /*std::cout << "v_attr \n " << v_attr << std::endl;
  std::cout << "v_rep \n " << v_rep << std::endl;
  std::cout << "v_tot \n " << v_tot << std::endl;
  std::cout << "new_position " << new_position << std::endl;*/
  
  getJointAnglesIndex(new_position);
 
}

void AllegroNodeGraspController::getJointAnglesThumb(Eigen::Matrix<long double, 3, 1> pos_thumb/*, Eigen::Matrix<long double, 3, 1> pos_ind*/) {

  double Pi = 2*asin(1);
  double Pi2 = 2*Pi;

  R2 << 1, 0, 0, 0,
        0, cos(current_state.position[12]), -sin(current_state.position[12]), 0,
        0, sin(current_state.position[12]),  cos(current_state.position[12]), 0,
        0,0,0,1;

  pos2 << pos_thumb(0,0), pos_thumb(1,0), pos_thumb(2,0), 1; 
  pos2 = (R2.inverse()) * (trans_th.inverse()) * pos2;
  pos2(1,0) = -pos2(1,0);  

  std::cout << "yeni pozzz \n" << pos2 << std::endl; 


  long double pos_x_th = pos2(0,0);
  long double pos_y_th = pos2(1,0);
  long double pos_z_th = pos2(2,0);

  Eigen::Matrix<long double, 3, 1> x_th;
  Eigen::Matrix<long double, 3, 1> x_new_th;
  Eigen::Matrix<long double, 3, 1> difference_th;


  //x_th << current_state.position[14], current_state.position[15];
  //x_new_th << current_state.position[14], current_state.position[15];

  x_th << 0.2,1.5, 0.2;
  x_new_th << 0.2,1.5, 0.2;

  std::cout << "x_th" << x_th << std::endl;

  std::cout << "teta1" << current_state.position[12] << std::endl;

  long double teta2;

  if( (pos_x_th-L1x_th) >= 0) 
    teta2 = atan((pos_y_th-L1y_th)/(pos_x_th-L1x_th));
  else 
    teta2 = Pi + atan((pos_y_th-L1y_th)/(pos_x_th-L1x_th));
  
  long double k3 = cos(teta2);
  long double k4 = sin(teta2);

  /*long double df11 =  L4_th*cos(x_th(0)+x_th(1))+L3_th*cos(x_th(0));
  long double df12 =  L4_th*cos(x_th(0)+x_th(1));
  long double df21 = -L4_th*sin(x_th(0)+x_th(1))-L3_th*sin(x_th(0));
  long double df22 = -L4_th*sin(x_th(0)+x_th(1));*/

  long double df11 =  -sin(x_th(0))*(L4_th*sin(x_th(1)+x_th(2)) + L3_th*sin(x_th(1)));
  long double df12 =   cos(x_th(0))*(L4_th*cos(x_th(1)+x_th(2)) + L3_th*cos(x_th(1)));
  long double df13 =   cos(x_th(0))*(L4_th*cos(x_th(1)+x_th(2)));

  long double df21 =  cos(x_th(0))*(L4_th*sin(x_th(1)+x_th(2)) + L3_th*sin(x_th(1)));
  long double df22 =  sin(x_th(0))*(L4_th*cos(x_th(1)+x_th(2)) + L3_th*cos(x_th(1)));
  long double df23 =  sin(x_th(0))*(L4_th*cos(x_th(1)+x_th(2)));

  long double df31 = 0;
  long double df32 = -L4_th*sin(x_th(1)+x_th(2))-L3_th*sin(x_th(1));
  long double df33 = -L4_th*sin(x_th(1)+x_th(2));

  //J_th << df11,df12,df21,df22;
  //J_th_inv = pseudoinverse2(J_th);

  J_th << df11, df12, df13, df21, df22, df23, df31, df32, df33;
  J_th_inv = pseudoinverse(J_th);

  while(true){

    x_th(0) = x_new_th(0);
    x_th(1) = x_new_th(1);
    x_th(2) = x_new_th(2);

    /*long double df11 =  L4_th*cos(x_th(0)+x_th(1))+L3_th*cos(x_th(0));
    long double df12 =  L4_th*cos(x_th(0)+x_th(1));
    long double df21 = -L4_th*sin(x_th(0)+x_th(1))-L3_th*sin(x_th(0));
    long double df22 = -L4_th*sin(x_th(0)+x_th(1));
    J_th << df11,df12,df21,df22;
    J_th_inv = pseudoinverse2(J_th);*/

    /*Eigen::Matrix<long double, 2, 1> fvec_th( ((pos_x_th-L1x_th)/k3)  - L4_th*sin(x_th(0)+x_th(1)) - L3_th*sin(x_th(0)) ,
                                             pos_z_th-L2_th - L1z_th - L4_th*cos(x_th(0)+x_th(1)) - L3_th*cos(x_th(0)));*/

    Eigen::Matrix<long double, 3, 1> fvec_th( pos_x_th - L1x_th - cos(x_th(0))*(L4_th*sin(x_th(1)+x_th(2)) + L3_th*sin(x_th(1))) ,
                                              pos_y_th - L1y_th - sin(x_th(0))*(L4_th*sin(x_th(1)+x_th(2)) + L3_th*sin(x_th(1))) ,
                                              pos_z_th - L2_th - L1z_th - L4_th*cos(x_th(1)+x_th(2)) - L3_th*cos(x_th(1)) );

    /*x_new_th(0) = x_th(0) - (J_th_inv(0,0)*fvec_th(0)+J_th_inv(0,1)*fvec_th(1));
    x_new_th(1) = x_th(1) - (J_th_inv(1,0)*fvec_th(0)+J_th_inv(1,1)*fvec_th(1));*/

    x_new_th = x_th - (J_th_inv*fvec_th);
   


    difference_th=x_new_th-x_th;

    if(difference_th.norm() < 1e-1) {
      std::cout << "posx =" << L1x_th + cos(x_th(0))*(L4_th*sin(x_th(1)+x_th(2))+L3_th*sin(x_th(1))) << std::endl;
      std::cout << "posy =" << L1y_th + sin(x_th(0))*(L4_th*sin(x_th(1)+x_th(2))+L3_th*sin(x_th(1))) << std::endl;
      std::cout << "posz =" << L2_th + L1z_th + L4_th*cos(x_th(1)+x_th(2)) + L3_th*cos(x_th(1)) << std::endl;

      double teta2 = std::fmod(x_th(0),Pi2);
      double teta3 = std::fmod(x_th(1),Pi2);
      double teta4 = std::fmod(x_th(2),Pi2);
      std::cout << "posx' =" << L1x_th + k3*(L4_th*sin(std::fmod(x_th(0)+x_th(1),Pi2))+L3_th*sin(teta3)) << std::endl;
      std::cout << "posy' =" << L1y_th + k4*(L4_th*sin(std::fmod(x_th(0)+x_th(1),Pi2))+L3_th*sin(teta3)) << std::endl;
      std::cout << "posz' =" << L2_th  + L1z_th + L4_th*cos(std::fmod(x_th(0)+x_th(1),Pi2)) + L3_th*cos(teta3) << std::endl;

      std::cout << "teta2" << teta2 << std::endl;
      std::cout << "teta3" << teta3 << std::endl;
      std::cout << "toplam" << std::fmod(x_th(1)+x_th(2),Pi2) << std::endl;
      break;
    }
    //current_state
    //desired_state_pub.publish(current_state);
    
  }

}

void AllegroNodeGraspController::getJointAnglesIndex(Eigen::Matrix<long double, 3, 1> pos_index/*, Eigen::Matrix<long double, 3, 1> pos_ind*/) {

  R1_ind << cos(current_state.position[0]), -sin(current_state.position[0]), 0, 0,
            sin(current_state.position[0]), cos(current_state.position[0]), 0, 0,
            0, 0,  1, 0,
            0,0,0,1;


  //std::cout << "teta1" << current_state.position[0] << std::endl; 
  //std::cout << "R1_ind" << R1_ind << std::endl; 
  //std::cout << "trans_ind" << trans_ind << std::endl;        
  pos2 << pos_index(0,0), pos_index(1,0), pos_index(2,0), 1; 
  //pos2  <<  0.0535676, -0.0423956, -0.0511716,1;         
  pos2 = (R1_ind.inverse()) * (trans_ind.inverse()) * pos2;
  //pos2(1,0) = -pos2(1,0); 
  //std::cout << pos2 << std::endl;        

}

void AllegroNodeGraspController::moveToDesiredGraspType() {

  for (int i = 0; i < DOF_JOINTS; i++) {
    joint[i] = 0;
    stop_table[i] = 0;
  }

  smoothPositionControlling(desired_position);
  //getCurrentPosition();
}


void AllegroNodeGraspController::separateFingers(){

  double separated_position[DOF_JOINTS]; 

  for(int i = 0; i<DOF_JOINTS; i++) {
    if(i == 1 || i == 5 || i == 9) { //1 INDEX       5 MIDDLE       9 LITTLE      
      separated_position[i] = 0.60;
      joint[i] = 0;
    }
    else {
      separated_position[i] = current_state.position[i];
    }
    if(i == 14) { //14 THUMB
      separated_position[i] = 0.30;
      joint[i] = 0;
    }
    
  }

  smoothPositionControlling(separated_position);
}

void AllegroNodeGraspController::separateSmart() {
  double vector_a[DOF_JOINTS];
  double vector_height[DOF_JOINTS];
  //double unity_vector_a[DOF_JOINTS];
  //double unity_vector_height[DOF_JOINTS];

  double *ptrVector_a, *ptrVector_height;
  ptrVector_a = vector_a;
  ptrVector_height = vector_height;
  findVectors(ptrVector_a, ptrVector_height);

  /*for (int i = 0; i < DOF_JOINTS; ++i)
  {
    std::cout << "vector height " << vector_height[i] << std::endl;
  }*/

  /*for (int i = 0; i < DOF_JOINTS; ++i)
  {
    unity_vector_a[i] = vector_a[i]/euclideanNorm(vector_a);
    std::cout << "unity_vector_a " << unity_vector_a[i] << std::endl;
    unity_vector_height[i] = vector_height[i]/euclideanNorm(vector_height);
    std::cout << "unity_vector_height " << unity_vector_height[i] << std::endl;
  }*/

  double weight_a[11];
  double weight_h[11];
  weight_a[0] = -1.0;
  weight_h[0] = 0.0;
  double increment = 0.1;
  for(int i =0; i<10; i++) {
    weight_a[i+1] = weight_a[i] + increment;
    weight_h[i+1] = weight_h[i] + increment;
  }

  for (int i = 0; i < 11; ++i)
  {
    std::cout << "weight_h " << weight_h[i] << std::endl;
  }
  

  double intermediatePositons[DOF_JOINTS];
  for(int j =0; j<11; j++) {
    //std::cout << "vector height " << vector_height[i] << std::endl;
    for (int i = 0; i<DOF_JOINTS; i++)
    {
      intermediatePositons[i] = current_state.position[i] + (weight_a[j]*vector_a[i]/2   + weight_h[j]*vector_height[i]);
    }
    smoothPositionControlling(intermediatePositons);
  }
    

}

void AllegroNodeGraspController::separateSmartly() {
  double intermediatePositons[DOF_JOINTS];
  

  double decrement[DOF_JOINTS];

  getCurrentPosition(intermediatePositons);
  difference << (pos_th_middle(0,0)-pos_ind_middle(0,0)),
                (pos_th_middle(1,0)-pos_ind_middle(1,0)),
                (pos_th_middle(2,0)-pos_ind_middle(2,0));
  d_0 = difference.norm();

  for (int i = 0; i < DOF_JOINTS; ++i)
  { 
    intermediatePositons[i] = current_state.position[i];
    decrement[i] =  current_state.position[i];
  }
  

  while (true)  {
    for (int i = 0; i < DOF_JOINTS; ++i) {
      intermediatePositons[i] = intermediatePositons[i] - decrement[i]/10;
    }

    getCurrentPosition(intermediatePositons);
    difference << (pos_th_middle(0,0)-pos_ind_middle(0,0)),
                (pos_th_middle(1,0)-pos_ind_middle(1,0)),
                (pos_th_middle(2,0)-pos_ind_middle(2,0));
    d_0 = difference.norm();
    if(d_0 >= 0.05) {
      std::cout << "erw" <<std::endl;
      break;
    }   
  }            

  smoothPositionControlling(intermediatePositons);

}

void AllegroNodeGraspController::findVectors(double *ptrVect_a, double *ptrVect_height) {
  double vect_a[DOF_JOINTS];
  double vect_b[DOF_JOINTS];
  double vect_c[DOF_JOINTS];
  double vect_k[DOF_JOINTS];
  
  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    *(ptrVect_a + i) = current_state.position[i] - initial_position[i];
    vect_a[i] = *(ptrVect_a + i);
    vect_b[i] = desired_position[i] - initial_position[i];
    vect_c[i] = desired_position[i] - current_state.position[i];
  }

  double norm_a = euclideanNorm(vect_a);
  double norm_b = euclideanNorm(vect_b);
  double norm_c = euclideanNorm(vect_c);

  double s = semiPerimeter(norm_a, norm_b, norm_c); //semiperimeter
  double area = triangleSurface(s, norm_a, norm_b, norm_c);
  double height = 2*area/norm_b;

  double norm_k = sqrt(pow(norm_a,2)-pow(height,2));

  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    vect_k[i] = norm_k * (vect_b[i]/norm_b);
  }

  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    *(ptrVect_height + i) = vect_k[i] - vect_a[i];
  }

  /*for (int i = 0; i < DOF_JOINTS; ++i)
  {
    std::cout << "vector height " << vector_height[i] << std::endl;
  }*/


}

double AllegroNodeGraspController::euclideanNorm(double vector[]) {

  double norm = 0.0;
  for (int i = 0; i < DOF_JOINTS; ++i)
  {
    norm += pow(vector[i],2);
  }
  norm = sqrt(norm);
  return norm;
}

double AllegroNodeGraspController::semiPerimeter(double norm_a, double norm_b, double norm_c) {

  double s;
  s = (norm_a + norm_b + norm_c)/2;
  return s;
}

double AllegroNodeGraspController::triangleSurface(double s, double norm_a, double norm_b, double norm_c) {
  double area;
  area = s*(s-norm_a)*(s-norm_b)*(s-norm_c);
  area = sqrt(area);
  return area;
}

/*double AllegroNodeGraspController::triangleHeight(double area, double norm_b) {
  double height;
  height = 2*area/norm_b;
  return height;
}*/

/*double angleBetweenAandB(double norm_a, double norm_b, double norm_c) {
  double angle;
  norm_a = pow(norm_a,2);
  norm_b = pow(norm_b,2);
  norm_c = pow(norm_c,2);
  angle = acos((pow(norm_a,2) + pow(norm_b,2) + pow(norm_c,2))/(2*norm_a*norm_b));
  return angle;
}*/





void AllegroNodeGraspController::smoothPositionControlling(double final_position[]) {

  double distance[DOF_JOINTS] = {0.0};

  double time_interval[2] = {-5.0 , 5.0};
  //sampling_rate = sampling_rate/velocity;
  double time_increment = (time_interval[1]-time_interval[0])/sampling_rate;
  double time_samples[sampling_rate+1];
  time_samples[0] = time_interval[0];
  for(int i =0; i<sampling_rate; i++)
    time_samples[i+1] = time_samples[i] + time_increment;

  
  double sigmoid_samples[DOF_JOINTS][sampling_rate+1];
  for (int i = 0; i < DOF_JOINTS; i++) {
    distance[i] = final_position[i] - current_state.position[i];
  }


  std::ofstream myfile("AnglesSmooth.txt");
  for(int i = 0; i<DOF_JOINTS; i++) {
    if(stop_table[i] == 0) {
      for(int j = 0; j<sampling_rate+1; j++) {
        sigmoid_samples[i][j] = sigmoidFunction(current_state.position[i], distance[i], /*1.0,*/ time_samples[j]);
        if (myfile.is_open()) 
          myfile << int(i) << " " << sigmoid_samples[i][j] <<"\n";         
        else
          std::cout << "Unable to open file";
        }  
    }
    else if(stop_table[i] == 1) {
      continue;
    }
  }


  
  int j = 0;
  ros::Rate rate(1000);  
  while(j<sampling_rate+1) {
    for(int i = 0; i<DOF_JOINTS; i++) {
      if(stop_table[i] == 0)
        current_state.position[i] =  sigmoid_samples[i][j];
      else if(stop_table[i] == 1)
        current_state.position[i] =  current_state.position[i];
    }
    desired_state_pub.publish(current_state);
    j = j+1;
    ros::spinOnce();
    rate.sleep();
  }

}

double AllegroNodeGraspController::sigmoidFunction(double initial_position, double distance, /*double velocity,*/ double time_sample) {
  //std::cout << "sigmoid velocity:" << velocity << std::endl;
  double y = initial_position + (distance/(1+exp((-velocity)*time_sample)));
  return y;
}


void AllegroNodeGraspController::moveToDesiredGraspType2() { 

  double distance[DOF_JOINTS] = {0.0};

  for (int i = 0; i < DOF_JOINTS; i++) {
    distance[i] = desired_position[i] - current_state.position[i];
    current_state.velocity[i] = (distance[i]/30000);
    joint[i] = 0;
    stop_table[i] = 0;
  }

  startclosing = true;


  std::ofstream myfile("AnglesNotSmooth.txt");
  //int iter = 0;//for txt file
  bool op = true;
  while(op){

    for (int i = 0; i < DOF_JOINTS; ++i)
    {
      if (myfile.is_open()) 
        myfile << current_state.position[i] <<"\n";         
      else
        std::cout << "Unable to open file";
    }
    

    updateCurrentPosition();

    if(checkEquality(joint))
      break; 
    
    desired_state_pub.publish(current_state);
    usleep(10);
  }

}


void AllegroNodeGraspController::updateCurrentPosition() {
  

  for (int i = 0; i < (int)DOF_JOINTS; i++) {
    if (stop_table[i] == 1)
      current_state.velocity[i] = 0.0; 
  }

  for (int i = 0; i < (int)DOF_JOINTS; i++) 
    current_state.position[i] = current_state.position[i] + current_state.velocity[i]; 

  if(startclosing) {  //just to fill the table only once
    for (int i = 0; i < DOF_JOINTS; i++) {
      if (current_state.position[i] <= desired_position[i]) 
        desiredisgreater[i] = 1;
      else 
        desiredisgreater[i] = 0;
    }

    startclosing = false; //just to fill the table only once
  }

  for (int i = 0; i < DOF_JOINTS; i++) {
    if (desiredisgreater[i] == 1 && current_state.position[i] >= desired_position[i]) 
      joint[i] = 1;
    else if (desiredisgreater[i] == 0 && current_state.position[i] <= desired_position[i]) 
      joint[i] = 1;
  }  

}



bool AllegroNodeGraspController::checkEquality(int array[]) {
  for (int i = 0; i < DOF_JOINTS; i++) {
    if(array[i] != 1)
      return false;
  }
  return true;
}

float AllegroNodeGraspController::average(int sum, int number) {

  float avg;

  avg = sum / number;

  return avg;
}

void AllegroNodeGraspController::initControllerxx() {
  current_state.position.resize(DOF_JOINTS);
  current_state.velocity.resize(DOF_JOINTS);
  
  desired_state.position.resize(DOF_JOINTS);
  desired_state.velocity.resize(DOF_JOINTS);

  q.w() = -0.477714;
  q.x() = -0.5213334;
  q.y() = 0.5213334;
  q.z() = -0.477714; 

  q1.w() = 0.999048;
  q1.x() = 0.0436194;
  q1.y() = 0;
  q1.z() = 0; 

  quat_to_rot = q.normalized().toRotationMatrix(); 
  quat1_to_rot = q1.normalized().toRotationMatrix(); 

  trans_th << quat_to_rot(0,0),quat_to_rot(0,1),quat_to_rot(0,2),-0.0182,
              quat_to_rot(1,0),quat_to_rot(1,1),quat_to_rot(1,2),-0.019333,
              quat_to_rot(2,0),quat_to_rot(2,1),quat_to_rot(2,2),-0.046687,
              0,0,0,1;

  trans_ind <<quat1_to_rot(0,0),quat1_to_rot(0,1),quat1_to_rot(0,2),0,
              quat1_to_rot(1,0),quat1_to_rot(1,1),quat1_to_rot(1,2),-0.0435,
              quat1_to_rot(2,0),quat1_to_rot(2,1),quat1_to_rot(2,2),-0.002242,
              0,0,0,1;            

  first_run = true;
}

void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}

/*void AllegroNodeGraspController::openHand() {

  int negative_table[16];
  int openHandPosition[16];

  for(int i = 0; i<DOF_JOINTS; i++) {
    if (current_state.position[i] <= 0) 
      negative_table[i] = 1;
    else 
      negative_table[i] = 0;
  }

  for(int i = 0; i<DOF_JOINTS; i++) {
    if(negative_table[i] == 1)
      openHandPosition[i] = current_state.position[i] + 0.20;
    else
      openHandPosition[i] = current_state.position[i] - 0.20;
  }
 
  ros::Rate rate(10000);
  while(true) {
    tnow = ros::Time::now();
    tnowsec = tnow.toSec();
    tnowsec_squared = pow(tnowsec,2.0);
    //dt = 1e-9 * (tnowsec_squared - tstartsec_squared).nsec; 
    dt = 1e-9 * (tnowsec_squared - tstartsec_squared);
    //std::cout << dt << std::endl;

    tstart = tnow;
    tstartsec = tstart.toSec();
    tstartsec_squared = pow(tstartsec,2.0);


    for (int i = 0; i < (int)DOF_JOINTS; i++) {
      if(negative_table[i] == 1) {
        current_state.position[i] = current_state.position[i] + (acc * dt/2); 
        //std::cout <<  current_state.position[i] << std::endl;
      }
      else
        current_state.position[i] = current_state.position[i] - (acc * dt/2);
    }

    for (int i = 0; i < (int)DOF_JOINTS; i++) {
      if (negative_table[i] == 1 && current_state.position[i] >= desired_position[i]) 
        joint[i] = 1;
      else if (negative_table[i] == 0 && current_state.position[i] <= desired_position[i]) 
        joint[i] = 1;
    } 

    if(checkEquality(joint))
      break;

    desired_state_pub.publish(current_state);
    rate.sleep();
  }
  
}

bool AllegroNodeGraspController::checkSeparate(int array[]) {
  
  if(array[1] != 1 || array[5] != 1 || array[9] != 1 || array[14] != 1)
    return false;
  
  return true;
}*/