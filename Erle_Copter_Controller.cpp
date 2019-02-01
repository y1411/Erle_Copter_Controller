#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Twist.h>

class ErleCopterManager
{
public:
 
  ErleCopterManager();
  ~ErleCopterManager();
  bool init(int argc, char** argv);

  int getRollVelocity();
  int getPitchVelocity();
  int getThrottleVelocity();
  int getYawVelocity();

private:
  int Roll, Pitch , Throttle , Yaw;

  int Roll_vel_step,Roll_vel_sleep ,Roll_vel_max , Roll_vel_min;
  int Pitch_vel_step,Pitch_vel_sleep, Pitch_vel_max, Pitch_vel_min;
  int Throttle_vel_step, Throttle_vel_max , Throttle_vel_min;
  int Yaw_vel_step,Yaw_vel_sleep, Yaw_vel_max, Yaw_vel_min;
  std::string name;

  void incrementRoll();
  void decrementRoll();
  void sleepRoll();
  void incrementPitch();
  void decrementPitch();
  void sleepPitch();
  void incrementThrottle();
  void decrementThrottle();
  void incrementYaw();
  void decrementYaw();
  void sleepYaw();
  void resetVelocity();
  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;

};

int ErleCopterManager::getRollVelocity()
{
  return Roll;
}

int ErleCopterManager::getPitchVelocity()
{
  return Pitch;
}

int ErleCopterManager::getThrottleVelocity()
{
  return Throttle;
}

int ErleCopterManager::getYawVelocity()
{
  return Yaw;
}

/**
 * @brief Default constructor, needs initialisation.
 */
ErleCopterManager::ErleCopterManager() :
                         Roll_vel_step(10),
			 Roll_vel_sleep(5),
                         Roll_vel_max(2000),
                         Roll_vel_min(1000),
                         Pitch_vel_step(10),
			 Pitch_vel_sleep(5),
                         Pitch_vel_max(2000),
                         Pitch_vel_min(1000),
                         Throttle_vel_step(10),
                         Throttle_vel_max(2000),
                         Throttle_vel_min(1000),
                         Yaw_vel_step(30),
                         Yaw_vel_sleep(5),
                         Yaw_vel_max(2000),
                         Yaw_vel_min(1000),
                         quit_requested(false),
                         key_file_descriptor(0),
                         Roll(0.0), Pitch(0.0),Throttle(0.0),Yaw(0.0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

ErleCopterManager::~ErleCopterManager()
{
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool ErleCopterManager::init(int argc, char** argv)
{
    std::cout << "Automatic!!" << std::endl;

  std::cout << "ErleCopterManager : Roll  vel step [" << Roll_vel_step << "]." << std::endl;
  //std::cout << "ErleCopterManager : using linear  vel max  [" << Roll_vel_max << "]." << std::endl;
  std::cout << "ErleCopterManager : Pitch vel step [" << Pitch_vel_step << "]." << std::endl;
  //std::cout << "ErleCopterManager : using angular vel max  [" << angular_vel_max << "]." << std::endl;
  std::cout << "ErleCopterManager : Throttle vel step [" << Throttle_vel_step << "]." << std::endl;
  std::cout << "ErleCopterManager : Yaw vel step [" << Yaw_vel_step << "]." << std::endl;

  Roll = atoi(argv[0]);
  Pitch = atoi(argv[1]);
  Throttle = atoi(argv[2]);
  Yaw = atoi(argv[3]);

  boost::thread t(boost::bind(&ErleCopterManager::keyboardInputLoop, this));
  return true;
}

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void ErleCopterManager::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("forward is keyboard type 'f'");
  puts("up is keyboard type 'u'");
  puts("Turn right is keyboard type 'r'");
  puts("Turn left is keyboard type 'l'");
  puts("Takeoff is keyboard type 't'");
  puts("down is keyboard type 'd' ");
  puts("q : quit.");
  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }

  puts("Exit");
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void Arming(int argc , char** argv)
{
  ros::init(argc, argv, "mavros_arming");
  ros::NodeHandle n;

  int rate = 10;
  ros::Rate r(rate);

  ////////////////////////////////////////////
  ///////////////////ARM//////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/\
mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(arming_cl.call(srv)){
    puts("Arming ok.");
  }
  
  // sleep(2);

}

void Setmode(int argc , char** argv)
{
  ros::init(argc, argv, "mavros_change_mode");
  ros::NodeHandle n;

  int rate = 10;
  ros::Rate r(rate);

  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode")\
    ;
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0;
  srv_setMode.request.custom_mode = "STABILIZE";
  if(cl.call(srv_setMode)){
    ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
  }
  sleep(2);

}

void ErleCopterManager::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
  case 't':
    {
      for(int i = 0 ; i < 6 ; i++){
	puts("take_off");
	incrementThrottle();
	sleep(1);
      }
      break;
    }
  case 'r':
    {
      puts("Turn right");
      incrementYaw();
      sleepYaw();
      decrementYaw();
      break;
    }
    case 'l':
    {
      puts("Turn left");
      decrementYaw();
      sleepYaw();
      incrementYaw();
      break;
    }
    case 'f':
    {
      puts("Forward");
      incrementPitch();
      sleepPitch();
      decrementPitch();
      break;
    }
    case 'u':
    {
      puts("up");
      incrementThrottle();
      break;
    }
    case 'd':
    {
      puts("down");
      decrementThrottle();
      break;
    }
    case 'q':
    {
      quit_requested = true;
      break;
    }
    default:
    {
      break;
    }
  }
}



void ErleCopterManager::incrementRoll()
{
  if (Roll < Roll_vel_max){
    Roll += Roll_vel_step;
  }
}

void ErleCopterManager::decrementRoll()
{
  if (Roll > Roll_vel_min){
    Roll -= Roll_vel_step;
  }
}

void ErleCopterManager::sleepRoll()
{
  sleep(Roll_vel_sleep);
}

void ErleCopterManager::incrementPitch()
{
  if (Pitch < Pitch_vel_max){
    Pitch += Pitch_vel_step;
  }
}


void ErleCopterManager::decrementPitch()
{
  if (Pitch > Pitch_vel_min){
    Pitch -= Pitch_vel_step;
  }
}

void ErleCopterManager::sleepPitch()
{
  sleep(Pitch_vel_sleep);
}


void ErleCopterManager::incrementThrottle()
{
  if (Throttle < Throttle_vel_max){
    Throttle += Throttle_vel_step;
  }
}

void ErleCopterManager::decrementThrottle()
{
  if (Throttle > Throttle_vel_min){
    Throttle -= Throttle_vel_step;
  }
}

void ErleCopterManager::incrementYaw()
{
  if (Yaw < Yaw_vel_max){
    Yaw += Yaw_vel_step;
  }
}

void ErleCopterManager::decrementYaw()
{
  if (Yaw > Yaw_vel_min){
    Yaw -= Yaw_vel_step;
  }
}

void ErleCopterManager::sleepYaw()
{
  sleep(Yaw_vel_sleep);
}

void ErleCopterManager::resetVelocity()
{
  Roll = 1500;
  Pitch = 1500;
  Throttle = 1500;
  Yaw = 1500;
}

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

ros::Publisher pub_cmd_vel;

int sterring = 0;

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  double sterring_percentage = 0;
  if(msg->angular.z!=0.0){
    sterring = msg->angular.z;
    sterring_percentage = msg->angular.z/0.2;
 
    sterring = -400 * sterring_percentage + 1500; 
  }else{
    sterring = 1500;
  }

  if(sterring>1900)
    sterring = 1900;

  if(sterring<1100)
    sterring = 1100;


  printf("->%.2f  %.2f %d\n",msg->angular.z, sterring_percentage, sterring);
}

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  ros::init(argc, argv, "mavros_rc_override");
  ros::NodeHandle n;
  
  
 
  ErleCopterManager erleCopter_manager;
  erleCopter_manager.init(argc , argv);

  int rate = 100;
  ros::Rate r(rate);

  ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
  mavros_msgs::OverrideRCIn msg_override;

  ros::Subscriber sub2 = n.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 1000, cmd_vel_Callback);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(100);
  async_spinner.start();
  Setmode(argc , argv);
  Arming(argc , argv);
  while (n.ok()){

        msg_override.channels[0] = erleCopter_manager.getRollVelocity() ;
        msg_override.channels[1] = erleCopter_manager.getPitchVelocity();
        msg_override.channels[2] = erleCopter_manager.getThrottleVelocity();
        msg_override.channels[3] = erleCopter_manager.getYawVelocity();
        msg_override.channels[4] = 1100;
        msg_override.channels[5] = 1500;
        msg_override.channels[6] = 1500;
        msg_override.channels[7] = 1100;

        rc_override_pub.publish(msg_override);
        r.sleep();
  }

  return 0;
}
