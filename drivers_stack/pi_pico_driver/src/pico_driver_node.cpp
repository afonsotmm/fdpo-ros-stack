#include "pico_driver_node.h"

PiPicoDriver::PiPicoDriver(ros::NodeHandle& nh_) : nh(nh_) {
    velSub = nh.subscribe("/cmd_vel", 10, &PiPicoDriver::velCallBack, this);
    pickBoxSub = nh.subscribe("/pick_box", 10, &PiPicoDriver::PickBoxCallBack, this);
    posePub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    tofPub = nh.advertise<std_msgs::Bool>("/tof_measure", 10);
    startSerial("/dev/ttyACM0", 115200);
    //std::thread([this]() { this->readSerialLoop(); }).detach();
    
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
        
    if (serial_fd_ < 0) return;

    std::string cmd = "VEL:" + std::to_string(v_d) + "," + std::to_string(w_d);
    std::string response = sendCommandAndWait(cmd, 500);  // espera até 500ms

    ROS_INFO("Enviado para a Pico: %s", cmd.c_str()); 
    //ROS_INFO("Resposta da Pico: %s", response.c_str());  

}

void PiPicoDriver::PubPose(){
    
    geometry_msgs::Pose2D pose2d;
    pose2d.x = position.x;
    pose2d.y = position.y;
    pose2d.theta = position.theta;

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = pose2d.x;
    odom.pose.pose.position.y = pose2d.y;
    odom.pose.pose.position.z = 0.0;  // 2D, então z é zero

    // Converter theta (yaw) para quaternion
    tf::Quaternion q = tf::createQuaternionFromYaw(pose2d.theta);
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

    posePub.publish(odom);

}

void PiPicoDriver::PubTof(){
    
    std_msgs::Bool tofValue;
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
      DecodeMsg(msg.data);
    }
    usleep(7000); // 10 ms
  }
}

std::string PiPicoDriver::sendCommandAndWait(const std::string& cmd, int timeout_ms) {
    if (serial_fd_ < 0) {
        ROS_ERROR("Serial não está aberta.");
        return "";
    }

    // Limpa o buffer de entrada da serial
    tcflush(serial_fd_, TCIFLUSH);

    std::string command = cmd + "\n";
    ssize_t bytes_written = write(serial_fd_, command.c_str(), command.size());

    if (bytes_written < 0) {
        ROS_ERROR("Erro ao enviar comando para a serial.");
        return "";
    }

    //ROS_INFO("Comando enviado: %s", command.c_str());

    char buf[256];
    std::string response;

    // Tempo de início com relógio do ROS
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout_duration(timeout_ms / 1000.0);  // converte ms para segundos
    ros::Duration poll_interval(0.01);  // 10 ms entre leituras

    while (ros::Time::now() - start_time < timeout_duration) {
        int n = read(serial_fd_, buf, sizeof(buf) - 1);

        if (n > 0) {
            buf[n] = '\0';
            response += std::string(buf);

            // Se encontrar caractere de nova linha, assume fim da resposta
            size_t newline_pos = response.find('\n');
            if (newline_pos != std::string::npos) {
                response = response.substr(0, newline_pos);  // só pega até o \n
                //ROS_INFO("Recebido da Pico: %s", response.c_str());
                // DecodeMsg(response);
                return response;
            }
        }

        poll_interval.sleep();  // espera 10 ms
    }

    ROS_WARN("Timeout esperando resposta da Pico.");
    return "";
}

void PiPicoDriver::DecodeMsg(const std::string& msg) {
    //"POS: 12.5, -7.3, 90.0"

    // Cria uma cópia modificável da string
    std::string data = msg;

    // Verifica se é uma mensagem de posição
    if (msg.find("POS:") == 0) {
        std::string data = msg.substr(4);  // Remove "POS:"

        std::stringstream ss(data);
        double x, y, theta;
        char comma1, comma2;

        if (ss >> x >> comma1 >> y >> comma2 >> theta) {
            ROS_INFO("POS recebida: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
            // Aqui você pode salvar, publicar, ou usar os valores
            position.x = x;
            position.y = y;
            position.theta = theta;
            PubPose();
        } else {
            ROS_WARN("Erro ao fazer parse da mensagem POS: %s", msg.c_str());
        }

    } else if (msg.find("TOF:") == 0) {
        std::string data = msg.substr(4); // Remove "TOF:"
        
        // Remove espaços no início e fim
        data.erase(0, data.find_first_not_of(" \t"));
        data.erase(data.find_last_not_of(" \t") + 1);

        std::istringstream iss(data);
        if (iss >> Tof) {
            ROS_INFO("TOF recebida: %d", Tof);
            // Aqui você pode usar o valor
            PubTof();
        } else {
            ROS_WARN("Erro ao fazer parse da mensagem TOF: %s", msg.c_str());
        }
        
    } else {
    
      ROS_WARN("Mensagem desconhecida: %s", msg.c_str());
    }
    
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pico_driver_node");
  ros::NodeHandle nh;

  PiPicoDriver driver(nh);  // cria seu objeto controlador

  ros::spin();  // espera callbacks

  return 0;
}