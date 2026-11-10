// Onboard UWB + IMU estimator. Publishes local odometry/status only.
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <urdf/model.h>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <regex>
#include <iomanip>
#include "uwb_eskf/filter.hpp"
using uwb::M3;
using uwb::V3;

class UwbEskfNode : public rclcpp::Node {
 public:
  UwbEskfNode() : Node("uwb_eskf") {
    const auto share=ament_index_cpp::get_package_share_directory("uwb_eskf");
    confirmed_=declare_parameter("configuration_confirmed",false);
    config_sha_=declare_parameter("config_sha256",std::string(""));
    rig_=declare_parameter("rig_file",share+"/config/rig.txt");
    anchors_=uwb::load_rig(rig_);bias_=uwb::load_bias(rig_);
    if(confirmed_ && anchors_.size()<4)throw std::runtime_error("confirmed rig requires >= 4 anchors");
    lo_=hi_=V3::Zero();if(!anchors_.empty())lo_=hi_=anchors_.begin()->second;
    for(auto &[id,a]:anchors_){if(!a.allFinite())throw std::runtime_error("invalid anchor");lo_=lo_.cwiseMin(a);hi_=hi_.cwiseMax(a);}
    ignore_bias_=declare_parameter("ignore_rig_bias",false);
    mode_=declare_parameter("imu",std::string("none"));
    pr_.sig_r=declare_parameter("sig_r",pr_.sig_r);pr_.q_cv=declare_parameter("q_cv",pr_.q_cv);
    min_anchors_=declare_parameter("min_init_anchors",int(std::max<size_t>(4,std::min<size_t>(6,anchors_.size()))));
    init_window_=declare_parameter("init_window_s",.05);imu_timeout_=declare_parameter("imu_timeout_s",.2);
    range_timeout_=declare_parameter("range_timeout_s",.5);input_age_=declare_parameter("max_input_age_s",.25);
    static_s_=declare_parameter("init_static_s",1.0);yaw0_=declare_parameter("yaw0_deg",0.0)*M_PI/180;
    frame_=declare_parameter("frame_id",std::string("uwb_rig"));
    if(min_anchors_<4 || (confirmed_ && size_t(min_anchors_)>anchors_.size()) || pr_.sig_r<=0 || imu_timeout_<=0 || range_timeout_<=0 || input_age_<=0 || init_window_<=0 || static_s_<=0)
      throw std::runtime_error("invalid estimator thresholds");
    const auto urdf=declare_parameter("urdf_file",share+"/config/uni.urdf");
    const auto tag=declare_parameter("tag_link",std::string("uwb_tag_1"));
    body_=declare_parameter("imu_link",std::string(mode_=="px4" || mode_=="mavlink"?"imu_link":mode_=="d435i"?"camera_imu_frame":tag));
    const auto lv=declare_parameter("lever",std::vector<double>{});
    if(lv.size()==3)lever_=V3(lv[0],lv[1],lv[2]);
    else if(!lv.empty())throw std::runtime_error("lever requires 3 values in body FLU");
    else{urdf::Model m;if(!m.initFile(urdf))throw std::runtime_error("cannot read URDF");lever_=(link_pose(m,body_).inverse()*link_pose(m,tag)).translation();}
    if(!lever_.allFinite())throw std::runtime_error("invalid lever arm");
    if(mode_=="mavlink" || mode_=="d435i"){
      std::vector<double> rotation=mode_=="mavlink"?std::vector<double>{1,0,0,0,-1,0,0,0,-1}:std::vector<double>{0,0,1,-1,0,0,0,-1,0};
      rotation=declare_parameter("imu_R",rotation);
      if(rotation.size()!=9)throw std::runtime_error("imu_R requires 9 values");
      imu_R_=Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rotation.data());
      if(!imu_R_.allFinite() || (imu_R_.transpose()*imu_R_-M3::Identity()).norm()>1e-5 || std::abs(imu_R_.determinant()-1)>1e-5)throw std::runtime_error("imu_R must be a rotation");
      const auto topic=declare_parameter("imu_topic",std::string(mode_=="mavlink"?"/px4/imu_frd":"/camera/camera/imu"));
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        const auto &a=m->linear_acceleration,&w=m->angular_velocity;
        on_imu(rclcpp::Time(m->header.stamp).seconds(),imu_R_*V3(a.x,a.y,a.z),imu_R_*V3(w.x,w.y,w.z));});
    }else if(mode_=="px4"){
      px4_sub_=create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined",rclcpp::SensorDataQoS(),[this](px4_msgs::msg::SensorCombined::ConstSharedPtr m){
        const V3 flu(1,-1,-1);
        on_imu(now().seconds(),flu.cwiseProduct(V3(m->accelerometer_m_s2[0],m->accelerometer_m_s2[1],m->accelerometer_m_s2[2])),flu.cwiseProduct(V3(m->gyro_rad[0],m->gyro_rad[1],m->gyro_rad[2])));});
    }else if(mode_!="none")throw std::runtime_error("imu must be none | d435i | px4 | mavlink");
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("~/odom",10);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",10);
    discover_=create_wall_timer(std::chrono::seconds(1),[this]{discover();});
    pub_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{publish();});
    stats_=create_wall_timer(std::chrono::seconds(1),[this]{status();});
    RCLCPP_INFO(get_logger(),"%zu anchors, imu=%s, configuration_confirmed=%d, lever=(%.4f %.4f %.4f), ignore_rig_bias=%d",anchors_.size(),mode_.c_str(),confirmed_,lever_.x(),lever_.y(),lever_.z(),ignore_bias_);
  }
 private:
  bool eskf()const{return mode_!="none";}
  static bool geometry_ok(const std::vector<V3> &A){
    double span=0;for(const auto &a:A)for(const auto &b:A)span=std::max(span,(a-b).cwiseAbs().maxCoeff());
    for(size_t i=0;i<A.size();++i)for(size_t j=i+1;j<A.size();++j)for(size_t k=j+1;k<A.size();++k)for(size_t l=k+1;l<A.size();++l){
      M3 e;e.col(0)=A[j]-A[i];e.col(1)=A[k]-A[i];e.col(2)=A[l]-A[i];
      if(std::abs(e.determinant())>std::max(1e-9,span*span*span*1e-7))return true;
    }return false;
  }
  static Eigen::Isometry3d link_pose(const urdf::Model &m,const std::string &name){
    auto l=m.getLink(name);if(!l)throw std::runtime_error("URDF missing link "+name);
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    for(;l->parent_joint;l=l->getParent()){const auto &o=l->parent_joint->parent_to_joint_origin_transform;T=Eigen::Translation3d(o.position.x,o.position.y,o.position.z)*Eigen::Quaterniond(o.rotation.w,o.rotation.x,o.rotation.y,o.rotation.z)*T;}return T;
  }
  static std::string quote(const std::string &s){std::string out="\"";for(char c:s){if(c=='"'||c=='\\')out+='\\';if(c=='\n')out+="\\n";else out+=c;}return out+'"';}
  bool fresh_stamp(double t){double age=now().seconds()-t;return std::isfinite(t)&&age>=-.1&&age<input_age_;}
  void reset_position(const std::string &r){inited_=false;latest_.clear();last_range_good_=-1;rej_run_=0;reason_=r;}
  void reset_attitude(const std::string &r){reset_position(r);att_ready_=false;t0_static_=-1;n_static_=0;acc_sum_.setZero();gyr_sum_.setZero();t_=-1;}
  void discover(){
    static const std::regex re("^/uwb/anchor_(\\d+)/range$");
    for(const auto &[name,types]:get_topic_names_and_types()){
      std::smatch m;if(!std::regex_match(name,m,re)||range_subs_.count(name))continue;
      int id=std::stoi(m[1]);if(!anchors_.count(id)||std::find(types.begin(),types.end(),"sensor_msgs/msg/Range")==types.end())continue;
      range_subs_[name]=create_subscription<sensor_msgs::msg::Range>(name,rclcpp::QoS(100),[this,id](sensor_msgs::msg::Range::ConstSharedPtr r){on_range(id,rclcpp::Time(r->header.stamp).seconds(),r->range);});
    }
  }
  void on_imu(double t,const V3 &a,const V3 &w){
    if(!fresh_stamp(t)||!a.allFinite()||!w.allFinite()||(t_>=0&&t<=t_)){++invalid_imu_;return;}
    if(t_>=0&&t-t_>imu_timeout_)reset_attitude("IMU_GAP");
    last_imu_rx_=now().seconds();last_w_=w;++n_imu_;
    if(!att_ready_){
      if(std::abs(a.norm()-9.81)>.8||w.norm()>.15){t0_static_=-1;n_static_=0;acc_sum_.setZero();gyr_sum_.setZero();t_=t;return;}
      if(t0_static_<0)t0_static_=t;acc_sum_+=a;gyr_sum_+=w;++n_static_;
      if(t-t0_static_>=static_s_){R0_=uwb::attitude_from_accel(acc_sum_/n_static_,yaw0_);bg0_=gyr_sum_/n_static_;att_ready_=true;}
    }else if(inited_&&t_>=0){es_.predict(a,w,t-t_,pr_);if(!es_.p.allFinite()||!es_.P.allFinite())reset_attitude("NONFINITE_STATE");}
    t_=t;
  }
  void on_range(int id,double t,double r){
    if(!fresh_stamp(t)||!std::isfinite(r)||r<=0||r>100||(last_stamp_.count(id)&&t<=last_stamp_[id])){++invalid_range_;return;}
    last_stamp_[id]=t;last_range_rx_=now().seconds();++n_range_;
    if(!confirmed_)return;
    if(!ignore_bias_)if(auto b=bias_.find(id);b!=bias_.end())r-=b->second;
    if(r<=0){++invalid_range_;return;}
    if(eskf()&&(!att_ready_||last_imu_rx_<0||now().seconds()-last_imu_rx_>imu_timeout_||std::abs(t-t_)>imu_timeout_))return;
    if(!inited_){try_init(id,t,r);return;}
    double nis=0;bool ok;
    if(eskf())ok=es_.update(anchors_[id],r,lever_,pr_,nis);
    else{if(t>t_){cv_.predict(t-t_,pr_.q_cv);t_=t;}ok=cv_.update(anchors_[id],r,pr_,nis);}
    if(ok){++n_ok_;last_range_good_=now().seconds();rej_run_=0;}else if(++rej_run_>200)reset_position("RANGE_GATE_RESET");
  }
  void try_init(int id,double t,double r){
    latest_[id]={t,r};std::vector<V3>A;std::vector<double>rr;V3 c=V3::Zero();
    for(const auto &[i,tr]:latest_)if(t-tr.first>=0&&t-tr.first<init_window_){A.push_back(anchors_[i]);rr.push_back(tr.second);c+=anchors_[i];}
    if(A.size()<size_t(min_anchors_) || !geometry_ok(A))return;
    V3 p0=uwb::gauss_newton(A,rr,c/A.size());if(!p0.allFinite()){++invalid_init_;return;}
    double ss=0;for(size_t i=0;i<A.size();++i)ss+=std::pow((p0-A[i]).norm()-rr[i],2);double rms=std::sqrt(ss/A.size());
    if(!std::isfinite(rms)||rms>3*pr_.sig_r||(p0.array()<lo_.array()-2).any()||(p0.array()>hi_.array()+2).any()){++invalid_init_;return;}
    // GN estimates the TAG; the inertial state represents the IMU/body.
    if(eskf()){es_.init(p0-R0_*lever_,R0_);es_.bg=bg0_;}else{cv_.init(p0);t_=t;}
    inited_=true;last_range_good_=now().seconds();++initializations_;reason_="OK";
    RCLCPP_INFO(get_logger(),"initialized from %zu anchors, RMS %.4f m",A.size(),rms);
  }
  void publish(){
    if(!inited_||!confirmed_)return;double current=now().seconds();
    if(eskf()&&current-last_imu_rx_>imu_timeout_){reset_attitude("IMU_TIMEOUT");return;}
    if(current-last_range_good_>range_timeout_){reset_position("RANGE_TIMEOUT");return;}
    nav_msgs::msg::Odometry o;o.header.stamp=rclcpp::Time(int64_t(t_*1e9));o.header.frame_id=frame_;o.child_frame_id=body_;
    V3 p=eskf()?es_.p:V3(cv_.x.head<3>()),v=eskf()?V3(es_.R.transpose()*es_.v):V3(cv_.x.tail<3>());
    Eigen::Quaterniond q(eskf()?es_.R:M3::Identity());
    if(!p.allFinite()||!v.allFinite()||!q.coeffs().allFinite()){reset_attitude("NONFINITE_STATE");return;}
    o.pose.pose.position.x=p.x();o.pose.pose.position.y=p.y();o.pose.pose.position.z=p.z();
    o.pose.pose.orientation.w=q.w();o.pose.pose.orientation.x=q.x();o.pose.pose.orientation.y=q.y();o.pose.pose.orientation.z=q.z();
    o.twist.twist.linear.x=v.x();o.twist.twist.linear.y=v.y();o.twist.twist.linear.z=v.z();
    const V3 w=eskf()?V3(last_w_-es_.bg):V3::Zero();o.twist.twist.angular.x=w.x();o.twist.twist.angular.y=w.y();o.twist.twist.angular.z=w.z();
    const M3 vcov=eskf()?M3(es_.R.transpose()*es_.P.block<3,3>(3,3)*es_.R):M3(cv_.P.block<3,3>(3,3));
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
      o.pose.covariance[i*6+j]=eskf()?es_.P(i,j):cv_.P(i,j);o.pose.covariance[(i+3)*6+j+3]=eskf()?es_.P(6+i,6+j):(i==j?1e6:0);o.twist.covariance[i*6+j]=vcov(i,j);
      if(eskf()){o.pose.covariance[i*6+j+3]=es_.P(i,6+j);o.pose.covariance[(i+3)*6+j]=es_.P(6+i,j);}
    }odom_pub_->publish(o);++n_odom_;
  }
  void status(){
    std::string state=!confirmed_?"WAIT_CONFIGURATION":eskf()&&last_imu_rx_<0?"WAIT_IMU":eskf()&&!att_ready_?"WAIT_STATIC_IMU":!inited_?"WAIT_RANGES":"RUNNING";
    double current=now().seconds();int recent=0;for(auto &[id,t]:last_stamp_)if(current-t<init_window_)++recent;
    std::ostringstream s;s<<std::setprecision(12)<<"{\"state\":"<<quote(state)<<",\"reason\":"<<quote(reason_)<<",\"configuration_confirmed\":"<<(confirmed_?"true":"false")
      <<",\"imu\":"<<quote(mode_)<<",\"rig_file\":"<<quote(rig_)<<",\"config_sha256\":"<<quote(config_sha_)<<",\"configured_anchors\":"<<anchors_.size()<<",\"frame_id\":"<<quote(frame_)<<",\"body_frame\":"<<quote(body_)<<",\"ignore_rig_bias\":"<<(ignore_bias_?"true":"false")
      <<",\"lever_flu_m\":["<<lever_.x()<<","<<lever_.y()<<","<<lever_.z()<<"],\"imu_samples\":"<<n_imu_<<",\"range_samples\":"<<n_range_<<",\"accepted_ranges\":"<<n_ok_
      <<",\"odom_messages\":"<<n_odom_<<",\"initializations\":"<<initializations_<<",\"required_anchors\":"<<min_anchors_<<",\"recent_anchors\":"<<recent
      <<",\"invalid_imu\":"<<invalid_imu_<<",\"invalid_ranges\":"<<invalid_range_<<",\"invalid_initializations\":"<<invalid_init_
      <<",\"imu_age_s\":"<<(last_imu_rx_<0?-1:current-last_imu_rx_)<<",\"range_age_s\":"<<(last_range_rx_<0?-1:current-last_range_rx_)<<"}";
    std_msgs::msg::String msg;msg.data=s.str();status_pub_->publish(msg);
  }
  uwb::Params pr_;uwb::CV cv_;uwb::ESKF es_;
  std::map<int,V3>anchors_;std::map<int,double>bias_,last_stamp_;std::map<int,std::pair<double,double>>latest_;
  std::string rig_,mode_,frame_,body_,config_sha_,reason_="STARTING";
  bool confirmed_=false,ignore_bias_=false,inited_=false,att_ready_=false;
  int min_anchors_=6,n_static_=0,rej_run_=0;
  double init_window_,imu_timeout_,range_timeout_,input_age_,static_s_,yaw0_,t_=-1,t0_static_=-1,last_imu_rx_=-1,last_range_rx_=-1,last_range_good_=-1;
  V3 lever_,lo_,hi_,acc_sum_=V3::Zero(),gyr_sum_=V3::Zero(),bg0_=V3::Zero(),last_w_=V3::Zero();M3 imu_R_=M3::Identity(),R0_=M3::Identity();
  uint64_t n_imu_=0,n_range_=0,n_ok_=0,n_odom_=0,initializations_=0,invalid_imu_=0,invalid_range_=0,invalid_init_=0;
  std::map<std::string,rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>range_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr discover_,pub_timer_,stats_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UwbEskfNode>());rclcpp::shutdown();}
