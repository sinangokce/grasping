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
double distance[DOF_JOINTS] = {0.0};
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

  for (int i = 0; i < DOF_JOINTS; i++) {
    joint[i] = 0;
    stop_table[i] = 0;
  }

  if(first_run)
    moveToDesiredGraspType();
  else if(!first_run) {
    //openHand();
    //separateFingers();//to avoid collisions
    separateSmart();
    //moveToDesiredGraspType();
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

void AllegroNodeGraspController::moveToDesiredGraspType() {

  for (int i = 0; i < DOF_JOINTS; i++) {
    joint[i] = 0;
    stop_table[i] = 0;
  }

  smoothPositionControlling(desired_position);
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

  /*for (int i = 0; i<DOF_JOINTS; i++)
    std::cout << final_position[i] << std::endl;*/

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

  for(int i = 0; i<DOF_JOINTS; i++) {
    if(stop_table[i] == 0) {
      for(int j = 0; j<sampling_rate+1; j++)
        sigmoid_samples[i][j] = sigmoidFunction(current_state.position[i], distance[i], /*1.0,*/ time_samples[j]);
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



/*void AllegroNodeGraspController::updateCurrentPosition() {
  

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

void AllegroNodeGraspController::openHand() {

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
}

bool AllegroNodeGraspController::checkEquality(int array[]) {
  for (int i = 0; i < DOF_JOINTS; i++) {
    if(array[i] != 1)
      return false;
  }
  return true;
}*/

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

  first_run = true;
}

void AllegroNodeGraspController::doIt() {
  ros::Rate rate(250.0);
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
  }
}