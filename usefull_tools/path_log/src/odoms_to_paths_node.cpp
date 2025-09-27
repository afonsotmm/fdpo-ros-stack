#include <ros/ros.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
#include <string>
#include <vector>

struct RGBA { double r,g,b,a; };

struct Track {
  ros::Publisher path_pub;
  ros::Publisher marker_pub;
  nav_msgs::Path path;
  visualization_msgs::Marker line;
  ros::Time last_added;
};

class OdomsToPaths {
public:
  OdomsToPaths(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
  {
    // Parâmetros
    pnh_.param("topic_odom_filtered", t_odom_filtered_, std::string("/odometry/filtered"));
    pnh_.param("topic_odom",          t_odom_,          std::string("/odom"));
    pnh_.param("topic_ground_truth",  t_gt_,            std::string("/base_pose_ground_truth"));
    pnh_.param("fixed_frame",         fixed_frame_,     std::string("map"));      // onde desenhar
    pnh_.param("mode",                mode_,            std::string("marker"));   // "path" ou "marker"
    pnh_.param("max_len",             max_len_,         6000);                    // nº máximo de pontos
    pnh_.param("min_dt",              min_dt_,          0.05);                    // intervalo mínimo entre pontos (s)
    pnh_.param("line_width",          line_w_,          0.03);                    // espessura da linha (marker)

    // Cores por trilha
    odom_filtered_color_ = {0.10, 0.80, 0.95, 1.0}; // ciano
    odom_color_          = {0.95, 0.65, 0.10, 1.0}; // laranja
    gt_color_            = {0.10, 0.90, 0.20, 1.0}; // verde

    // Criar tracks/publicadores
    addTrack(t_odom_filtered_, "odom_filtered", odom_filtered_color_);
    addTrack(t_odom_,          "odom",          odom_color_);
    addTrack(t_gt_,            "ground_truth",  gt_color_);

    // Subscrições (passamos uma "key" para o callback identificar a trilha)
    if (!t_odom_filtered_.empty()) {
      subs_.push_back(
        nh_.subscribe<nav_msgs::Odometry>(
          t_odom_filtered_, 50,
          boost::bind(&OdomsToPaths::cbOdom, this, _1, std::string("odom_filtered"))
        )
      );
    }
    if (!t_odom_.empty()) {
      subs_.push_back(
        nh_.subscribe<nav_msgs::Odometry>(
          t_odom_, 50,
          boost::bind(&OdomsToPaths::cbOdom, this, _1, std::string("odom"))
        )
      );
    }
    if (!t_gt_.empty()) {
      subs_.push_back(
        nh_.subscribe<nav_msgs::Odometry>(
          t_gt_, 50,
          boost::bind(&OdomsToPaths::cbOdom, this, _1, std::string("ground_truth"))
        )
      );
    }

    ROS_INFO_STREAM("OdomsToPaths: mode=" << mode_
                    << " fixed_frame=" << fixed_frame_
                    << " topics=[" << t_odom_filtered_ << ", " << t_odom_
                    << ", " << t_gt_ << "]");
  }

private:
  ros::NodeHandle nh_, pnh_;
  std::vector<ros::Subscriber> subs_;
  std::unordered_map<std::string, Track> tracks_;

  std::string t_odom_filtered_, t_odom_, t_gt_, fixed_frame_, mode_;
  int max_len_;
  double min_dt_, line_w_;
  RGBA odom_filtered_color_, odom_color_, gt_color_;

  void addTrack(const std::string& topic, const std::string& ns, const RGBA& c)
  {
    if (topic.empty()) return;

    Track tr;
    tr.last_added = ros::Time(0);

    if (mode_ == "path") {
      // latch=true para o RViz ver o tópico assim que o nó arranca
      tr.path_pub = nh_.advertise<nav_msgs::Path>(topic + "/path", 1, /*latch=*/true);
      tr.path.header.frame_id = fixed_frame_;
      tr.path.header.stamp = ros::Time::now();
      tr.path.poses.clear();
      tr.path_pub.publish(tr.path); // primeira publicação (vazia)
    } else {
      tr.marker_pub = nh_.advertise<visualization_msgs::Marker>(topic + "/line", 1, /*latch=*/true);
      initLine(tr.line, ns, c);
      tr.line.header.stamp = ros::Time::now();
      tr.marker_pub.publish(tr.line); // primeira publicação (vazia)
    }

    tracks_.emplace(ns, std::move(tr));

    ROS_INFO("Tracking '%s' as '%s' -> publishing %s on %s/%s",
             topic.c_str(), ns.c_str(),
             (mode_ == "path" ? "nav_msgs/Path" : "Marker LINE_STRIP"),
             topic.c_str(), (mode_ == "path" ? "path" : "line"));
  }

  void initLine(visualization_msgs::Marker& m, const std::string& ns, const RGBA& c)
  {
    m.header.frame_id = fixed_frame_;
    m.header.stamp    = ros::Time(0);
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = line_w_; // espessura
    m.color.r = c.r; m.color.g = c.g; m.color.b = c.b; m.color.a = c.a;
    m.lifetime = ros::Duration(0.0);
    m.points.clear();
  }

  bool shouldAppend(Track& tr, const ros::Time& stamp)
  {
    if (!tr.last_added.isValid()) return true;
    return (stamp - tr.last_added).toSec() >= min_dt_;
  }

  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg, const std::string& key)
  {
    auto it = tracks_.find(key);
    if (it == tracks_.end()) return;
    Track& tr = it->second;

    const ros::Time stamp = msg->header.stamp;
    if (!shouldAppend(tr, stamp)) return;

    if (mode_ == "path") {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = stamp;
      ps.header.frame_id = fixed_frame_;          // desenhamos no fixed_frame
      ps.pose = msg->pose.pose;

      tr.path.header.stamp = stamp;
      tr.path.header.frame_id = fixed_frame_;
      tr.path.poses.push_back(ps);

      // limitar tamanho
      if ((int)tr.path.poses.size() > max_len_) {
        tr.path.poses.erase(tr.path.poses.begin(),
                            tr.path.poses.begin() + (tr.path.poses.size() - max_len_));
      }

      tr.last_added = stamp;
      tr.path_pub.publish(tr.path);
    } else {
      geometry_msgs::Point p;
      p.x = msg->pose.pose.position.x;
      p.y = msg->pose.pose.position.y;
      p.z = msg->pose.pose.position.z;

      tr.line.header.stamp = stamp;
      tr.line.points.push_back(p);

      // limitar tamanho
      if ((int)tr.line.points.size() > max_len_) {
        tr.line.points.erase(tr.line.points.begin(),
                             tr.line.points.begin() + (tr.line.points.size() - max_len_));
      }

      tr.last_added = stamp;
      tr.marker_pub.publish(tr.line);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odoms_to_paths_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OdomsToPaths node(nh, pnh);
  ros::spin();
  return 0;
}
