#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tua msg
#include <localizer/BeaconMatch.h>

static inline bool finite2(double x, double y) {
  return std::isfinite(x) && std::isfinite(y);
}

class VizMux {
public:
  VizMux(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_buffer_(), tf_listener_(tf_buffer_) {

    pnh.param<std::string>("target_frame", target_frame_, std::string("map"));
    pnh.param<std::string>("beacon_topic", beacon_topic_, std::string("beacon_Estimation"));
    pnh.param<std::string>("out_topic", out_topic_, std::string("viz_mux/markers"));

    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(out_topic_, 1);
    sub_beacons_ = nh.subscribe(beacon_topic_, 1, &VizMux::onBeaconEstimation, this);

    // (opcional) reencaminhar outros MarkerArray “como estão”
    // sub_other_ = nh.subscribe("other_markers_in", 1, &VizMux::onPassthroughMarkers, this);

    ROS_INFO("viz_mux: target_frame=%s, in=%s, out=%s",
             target_frame_.c_str(), beacon_topic_.c_str(), out_topic_.c_str());
  }

private:
  void onBeaconEstimation(const localizer::BeaconMatch::ConstPtr& msg) {
    visualization_msgs::MarkerArray arr;
    // DELETEALL
    { visualization_msgs::Marker clr; clr.action = visualization_msgs::Marker::DELETEALL; arr.markers.push_back(clr); }

    const std::string src_frame = msg->header.frame_id;   // ex.: "base_link"
    const ros::Time   stamp     = msg->header.stamp;

    geometry_msgs::TransformStamped T_dst_src;
    bool have_tf = true;
    try {
      T_dst_src = tf_buffer_.lookupTransform(target_frame_, src_frame, stamp, ros::Duration(0.2));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "viz_mux TF %s<- %s fail: %s",
                        target_frame_.c_str(), src_frame.c_str(), ex.what());
      have_tf = false;
    }

    int id = 0;
    for (const auto& cl : msg->clusters) {
      geometry_msgs::PointStamped p_src, p_dst;
      p_src.header.frame_id = src_frame;
      p_src.header.stamp    = stamp;

      // se no detector trocaste centroid -> centro_corrigido, aqui já vem “certo”
      p_src.point.x = cl.centroid.x;
      p_src.point.y = cl.centroid.y;
      p_src.point.z = 0.0;
      if (!finite2(p_src.point.x, p_src.point.y)) continue;

      if (have_tf) {
        try {
          tf2::doTransform(p_src, p_dst, T_dst_src);
        } catch (const tf2::TransformException& ex) {
          ROS_WARN_THROTTLE(1.0, "viz_mux doTransform fail: %s", ex.what());
          continue;
        }
      } else {
        p_dst = p_src; // desenha no frame de origem se TF falhar
      }

      // esfera do centro (corrigido) que o EKF deveria estar a usar
      visualization_msgs::Marker m;
      m.header.frame_id = have_tf ? target_frame_ : src_frame;
      m.header.stamp    = ros::Time::now();
      m.ns   = "beacon_center_meas";
      m.id   = id++;
      m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.pose.position = p_dst.point;
      m.scale.x = m.scale.y = m.scale.z = 0.10;
      m.color.r = 0.10; m.color.g = 0.95; m.color.b = 0.30; m.color.a = 1.0; // verde
      m.lifetime = ros::Duration(0.2);
      arr.markers.push_back(m);

      // etiqueta
      visualization_msgs::Marker label = m;
      label.ns = "beacon_center_label";
      label.id = id++;
      label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      label.pose.position.z += 0.25;
      label.scale.z = 0.18;
      label.color.r = label.color.g = label.color.b = 1.0;
      label.text = cl.beacon_match_name;
      arr.markers.push_back(label);
    }

    pub_markers_.publish(arr);
  }

  void onPassthroughMarkers(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    // exemplo de “mux”: reenviar outros markers como estão (cuidado com frames)
    pub_markers_.publish(*msg);
  }

  // members
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber sub_beacons_;
  ros::Subscriber sub_other_;
  ros::Publisher  pub_markers_;
  std::string target_frame_, beacon_topic_, out_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "viz_mux");
    ros::NodeHandle nh, pnh("~");
    VizMux node(nh, pnh);
    ros::spin();
    return 0;
}
