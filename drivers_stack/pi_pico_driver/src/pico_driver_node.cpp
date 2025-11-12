#include "pico_driver_node.h"

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {

  // ----------------------- ROS init -----------------------
  // -> Subs
  velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
  pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::pickBoxCallBack, this);
  // -> Pubs
  posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
  detectBoxPub = nh.advertise<std_msgs::Bool>("/box_detection", 10);
  // -> Timer
  commTimer = nh.createTimer(ros::Duration(0.02), &PiPicoDriver::commTick, this);

  // ---------------------- Serial init ---------------------
  std::string serial_port;
  nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  ROS_INFO("[PiPicoDriver] Usando porta serial: %s", serial_port.c_str());
  
  serial_fd_ = -1;
  startSerial(serial_port);
  
}

PiPicoDriver::~PiPicoDriver() {

  if (serial_fd_ >= 0) close(serial_fd_);

}

void PiPicoDriver::startSerial(const std::string& port) {

  serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (serial_fd_ < 0) {
    ROS_ERROR("Erro ao abrir porta serial: %s", port.c_str());
    return;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(serial_fd_, &tty) != 0) {
    ROS_ERROR("Erro ao obter atributos da serial");
    return;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;

  // ---- BLOQUEANTE ----
  tty.c_cc[VMIN]  = 1;  // espera pelo menos 1 byte
  tty.c_cc[VTIME] = 1;  // timeout de 100 ms
  // --------------------

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR("Erro ao aplicar configurações na serial");
  }
}

void PiPicoDriver::velCallBack(const geometry_msgs::Twist::ConstPtr& msg) {

  messageToSend.v_d = msg->linear.x;
  messageToSend.w_d = msg->angular.z;

}

void PiPicoDriver::pickBoxCallBack(const std_msgs::Bool::ConstPtr& msg) {

  messageToSend.pick_box = msg->data; 
  
}

void PiPicoDriver::pubOdom() {

  ros::Time current_time = ros::Time::now();
  
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";

  odom.pose.pose.position.x = messageToReceive.odom_pos.x;
  odom.pose.pose.position.y = messageToReceive.odom_pos.y;
  odom.pose.pose.position.z = 0.0;

  tf::Quaternion q = tf::createQuaternionFromYaw(messageToReceive.odom_pos.theta);
  tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

  posePub.publish(odom);

  // Publish TF transform odom -> base_link
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = messageToReceive.odom_pos.x;
  odom_trans.transform.translation.y = messageToReceive.odom_pos.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom.pose.pose.orientation;

  tf_broadcaster.sendTransform(odom_trans);

}

void PiPicoDriver::pubBoxDetection(){
    
  std_msgs::Bool msg;
  msg.data = messageToReceive.box_detection;
  detectBoxPub.publish(msg);
    
}

std::string PiPicoDriver::syncCall(const std::string& cmd, int timeout_ms) {
  if (serial_fd_ < 0) {
    ROS_ERROR("Serial closed.");
    return "";
  }

  tcflush(serial_fd_, TCIFLUSH);

  const std::string command = cmd + "\n";
  if (write(serial_fd_, command.c_str(), command.size()) < 0) {
    ROS_ERROR("Erro ao enviar comando para a serial.");
    return "";
  }

  char buf[256];
  std::string response;

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout_duration(timeout_ms / 1000.0);
  ros::Duration poll_interval(0.01);

  while (ros::Time::now() - start_time < timeout_duration) {
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      response += std::string(buf);
      size_t newline_pos = response.find('\n');
      if (newline_pos != std::string::npos) {
        std::string line = response.substr(0, newline_pos);
        decodeMsg(line);
        return line;
      }
    }
  }

  ROS_WARN("Timeout esperando resposta da Pico.");
  return "";
}

void PiPicoDriver::decodeMsg(const std::string& msg) {
  if (msg.rfind("POS:", 0) == 0) {
    double x=0, y=0, theta=0;
    int tof = 0;

    // Tenta com TOF primeiro
    int n = std::sscanf(msg.c_str(), "POS: %lf, %lf, %lf, TOF: %d", &x, &y, &theta, &tof);
    if (n < 3) {
      // fallback: só POS
      n = std::sscanf(msg.c_str(), "POS: %lf, %lf, %lf", &x, &y, &theta);
      tof = messageToReceive.box_detection ? 1 : 0; // mantém anterior
    }
    if (n >= 3) {
      messageToReceive.odom_pos = {x, y, theta};
      pubOdom();
      if (msg.find("TOF:") != std::string::npos) {
        messageToReceive.box_detection = (tof != 0);
        pubBoxDetection();
      }
      return;
    }
    ROS_WARN("Erro ao fazer parse da mensagem POS: %s", msg.c_str());
    return;
  }

  if (msg.rfind("TOF:", 0) == 0) {
    // (mantém este ramo se a Pico por vezes enviar só TOF)
    std::string data = msg.substr(4);
    data.erase(0, data.find_first_not_of(" \t"));
    data.erase(data.find_last_not_of(" \t") + 1);
    int flag = 0;
    std::istringstream iss(data);
    if (iss >> flag) {
      messageToReceive.box_detection = (flag != 0);
      pubBoxDetection();
    } else {
      ROS_WARN("Erro ao fazer parse da mensagem TOF: %s", msg.c_str());
    }
    return;
  }

  // Ignora ruído opcionalmente:
  if (msg.rfind("ACK:", 0) == 0) return;

  ROS_WARN("Mensagem desconhecida: %s", msg.c_str());
}

void PiPicoDriver::commTick(const ros::TimerEvent&) {
  
  // 1) Build the command
  std::string cmd = "CMD:" + std::to_string(messageToSend.v_d) + "," +
                             std::to_string(messageToSend.w_d) + "," +
                                           (messageToSend.pick_box ? "1" : "0"); 

  // 2) Send and Wait for response
  ROS_INFO("Pi4 Message: %s", cmd.c_str());
  std::string resp = syncCall(cmd, 50);
  ROS_INFO("PiPico Message: %s", resp.c_str());
  ROS_INFO("-------------------");
  if (resp.empty()) {
    con_state.missed++;
    if (con_state.missed >= 2) con_state.link_ok = false;
    return;
  }
  con_state.missed = 0; con_state.link_ok = true;

}