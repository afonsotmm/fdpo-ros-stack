#include "pico_driver_node.h"


PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {
    velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
    pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::PickBoxCallBack, this);
    posePub = nh.advertise<geometry_msgs::Pose2D>("/odom", 10);
    tofPub = nh.advertise<std_msgs::Float32>("/tof_measure", 10);
    startSerial("/dev/ttyACM0", 115200);
    std::thread([this]() { this->readSerialLoop(); }).detach();
    
}


void PiPicoDriver::PickBoxCallBack(const std_msgs::Bool::ConstPtr& msg) {
    ROS_INFO_STREAM("Received pick_box msg: " << msg->data);

    iman = msg->data;

    /*
    if (serial_fd_ < 0) return;

    std::string cmd = msg->data + "\n";  // adiciona quebra de linha, se necessário
    write(serial_fd_, cmd.c_str(), cmd.size());

    ROS_INFO("Enviado para a Pico: %s", cmd.c_str());
    */
}

void PiPicoDriver::velCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO_STREAM("Received cmd_vel: linear=" << msg->linear.x << ", angular=" << msg->angular.z);

    v_d = msg->linear.x;
    w_d = msg->angular.z;

}

void PiPicoDriver::PubPose(){
    
    geometry_msgs::Pose2D XYTheta;
    XYTheta.x = position.x;
    XYTheta.y = position.y;
    XYTheta.theta = position.theta;
    posePub.publish(XYTheta);

}

void PiPicoDriver::PubTof(){
    
    std_msgs::Float32 tofValue;
    tofValue.data = Tof;
    tofPub.publish(tofValue);
    
}

PiPicoDriver::~PiPicoDriver() {
  if (serial_fd_ > 0)
    close(serial_fd_);
}

void PiPicoDriver::startSerial(const std::string& port, int baud) {
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
  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 1;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    ROS_ERROR("Erro ao aplicar configurações na serial");
  }
}

void PiPicoDriver::readSerialLoop() {
  char buf[256];
  while (ros::ok()) {
    int n = read(serial_fd_, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      std_msgs::String msg;
      msg.data = std::string(buf);
      ROS_INFO("Recebido da Pico: %s", buf);
    }
    usleep(10000); // 10 ms
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pico_driver_node");
  ros::NodeHandle nh;

  PiPicoDriver driver(nh);  // cria seu objeto controlador

  ros::spin();  // espera callbacks

  return 0;
}