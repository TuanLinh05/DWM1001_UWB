// ROS adapters, live freshness/configuration guards and publication around the ROS-free core.
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#ifdef UWB_ESKF_WITH_PX4_MSGS
#include <px4_msgs/msg/sensor_combined.hpp>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <iomanip>
#include <memory>
#include "uwb_eskf/estimator.hpp"

using uwb::M3;using uwb::V3;using Core=uwb::Estimator;

class UwbEskfNode : public rclcpp::Node {
 public:
  UwbEskfNode():Node("uwb_eskf"){
    const auto share=ament_index_cpp::get_package_share_directory("uwb_eskf");
    confirmed_=declare_parameter("configuration_confirmed",false);
    config_sha_=declare_parameter("config_sha256",std::string(""));
    rig_=declare_parameter("rig_file",share+"/config/rig.txt");
    anchors_=uwb::load_rig(rig_);bias_=uwb::load_bias(rig_);
    if(confirmed_ && anchors_.size()<4)throw std::runtime_error("confirmed rig requires >=4 anchors");
    for(const auto &[id,a]:anchors_)if(!a.allFinite())throw std::runtime_error("invalid anchor");
    ignore_bias_=declare_parameter("ignore_rig_bias",false);
    mode_=declare_parameter("imu",std::string("none"));
    frame_=declare_parameter("frame_id",std::string("uwb_rig"));
    yaw0_=declare_parameter("yaw0_deg",0.)*M_PI/180;
    params_.sig_r=declare_parameter("sig_r",.15);params_.q_cv=declare_parameter("q_cv",1.);
    params_.na=declare_parameter("na",.45);params_.ng=declare_parameter("ng",.016);
    params_.nba=declare_parameter("nba",.003);params_.nbg=declare_parameter("nbg",.0001);
    params_.gate=declare_parameter("gate",25.);params_.k_pos=declare_parameter("k_pos",1.5);params_.k_neg=declare_parameter("k_neg",3.);
    params_.init_min_anchors=declare_parameter("min_init_anchors",int(std::max<size_t>(4,std::min<size_t>(6,anchors_.size()))));
    params_.init_window_s=declare_parameter("init_window_s",.05);params_.init_static_s=declare_parameter("init_static_s",1.);
    imu_timeout_=declare_parameter("imu_timeout_s",.2);range_timeout_=declare_parameter("range_timeout_s",.5);
    input_age_=declare_parameter("max_input_age_s",.25);params_.imu_gap_reset_s=imu_timeout_;
    params_.min_anchor_interval_s=declare_parameter("min_anchor_interval_s",.02);
    params_.imu_average_s=declare_parameter("imu_average_s",.02);
    params_.reinit_keep_attitude=declare_parameter("reinit_keep_attitude",true);
    params_.reinit_after_rejects=declare_parameter("reinit_after_rejects",200);
    params_.static_check=true;
    params_.static_acc_tol=declare_parameter("static_acc_tol",.8);params_.static_gyro_tol=declare_parameter("static_gyro_tol",.15);
    use_attitude_=declare_parameter("use_px4_attitude",1);
    if(use_attitude_!=0 && use_attitude_!=1)throw std::runtime_error("use_px4_attitude must be 0 or 1");
    params_.attitude_init=use_attitude_?"external":"static";
    const double tilt=declare_parameter("att_sig_tilt_deg",2.),yaw=declare_parameter("att_sig_yaw_deg",5.);
    params_.att_sig_deg=V3(tilt,tilt,yaw);params_.att_gate=declare_parameter("att_gate",25.);
    params_.att_min_interval_s=declare_parameter("att_min_interval_s",.02);
    if(params_.init_min_anchors<4 || (confirmed_&&size_t(params_.init_min_anchors)>anchors_.size())
       || !std::isfinite(yaw0_) || params_.init_window_s<=0 || params_.init_static_s<=0 || params_.reinit_after_rejects<1
       || !std::isfinite(imu_timeout_)||imu_timeout_<=0||!std::isfinite(range_timeout_)||range_timeout_<=0
       || !std::isfinite(input_age_)||input_age_<=0||!std::isfinite(params_.q_cv)||params_.q_cv<=0)
      throw std::runtime_error("invalid estimator thresholds");
    for(double noise:{params_.na,params_.ng,params_.nba,params_.nbg})if(!std::isfinite(noise)||noise<0)throw std::runtime_error("invalid IMU process noise");
    const auto tag=declare_parameter("tag_link",std::string("uwb_tag_1"));
    body_=declare_parameter("imu_link",std::string(mode_=="px4"||mode_=="mavlink"?"imu_link":mode_=="d435i"?"camera_imu_frame":tag));
    const auto lv=declare_parameter("lever",std::vector<double>{});
    if(lv.size()==3)lever_=V3(lv[0],lv[1],lv[2]);
    else if(!lv.empty())throw std::runtime_error("lever requires 3 FLU coordinates");
    else{
      urdf::Model m;const auto path=declare_parameter("urdf_file",share+"/config/uni.urdf");
      if(!m.initFile(path))throw std::runtime_error("cannot read URDF");
      lever_=(link_pose(m,body_).inverse()*link_pose(m,tag)).translation();
    }
    core_=std::make_unique<Core>(params_,anchors_,lever_,yaw0_);
    if(mode_=="mavlink" || mode_=="d435i"){
      auto rotation=declare_parameter("imu_R",mode_=="mavlink"?std::vector<double>{1,0,0,0,-1,0,0,0,-1}:std::vector<double>{0,0,1,-1,0,0,0,-1,0});
      if(rotation.size()!=9)throw std::runtime_error("imu_R requires 9 values");
      imu_R_=Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(rotation.data());
      if(!uwb::valid_rotation(imu_R_))throw std::runtime_error("imu_R must be a proper rotation");
      const auto topic=declare_parameter("imu_topic",std::string(mode_=="mavlink"?"/px4/imu_frd":"/camera/camera/imu"));
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::ConstSharedPtr m){
        const auto &a=m->linear_acceleration,&w=m->angular_velocity;
        on_imu(rclcpp::Time(m->header.stamp).nanoseconds(),imu_R_*V3(a.x,a.y,a.z),imu_R_*V3(w.x,w.y,w.z));
      });
    }else if(mode_=="px4"){
#ifdef UWB_ESKF_WITH_PX4_MSGS
      px4_sub_=create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined",rclcpp::SensorDataQoS(),[this](px4_msgs::msg::SensorCombined::ConstSharedPtr m){
        const V3 flip(1,-1,-1);
        on_imu(now().nanoseconds(),flip.cwiseProduct(V3(m->accelerometer_m_s2[0],m->accelerometer_m_s2[1],m->accelerometer_m_s2[2])),
               flip.cwiseProduct(V3(m->gyro_rad[0],m->gyro_rad[1],m->gyro_rad[2])));
      });
#else
      throw std::runtime_error("direct px4 input requires UWB_ESKF_WITH_PX4_MSGS=ON; mavlink input is available");
#endif
    }else if(mode_!="none")throw std::runtime_error("imu must be none | d435i | px4 | mavlink");
    attitude_topic_=declare_parameter("attitude_topic",std::string("/px4/attitude_frd"));
    if(eskf() && use_attitude_){
      att_sub_=create_subscription<geometry_msgs::msg::QuaternionStamped>(attitude_topic_,rclcpp::SensorDataQoS(),[this](geometry_msgs::msg::QuaternionStamped::ConstSharedPtr m){
        const auto stamp=rclcpp::Time(m->header.stamp).nanoseconds();
        if(!fresh_stamp(stamp)||m->header.frame_id!="px4_local_ned"){++invalid_attitude_;return;}
        const double t=core_time(stamp);
        if(t<=last_attitude_stamp_){++invalid_attitude_;return;}
        const auto &q=m->quaternion;const Eigen::Vector4d quat(q.w,q.x,q.y,q.z);
        try{
          // Offset is fixed for this node session, including after every IMU reset.
          if(!rig_yaw_offset_)rig_yaw_offset_=uwb::rig_yaw_offset_from_yaw0(quat,yaw0_);
          core_->on_attitude(t,uwb::px4_attitude_to_rig(quat,*rig_yaw_offset_));
          last_attitude_stamp_=t;
        }catch(const std::invalid_argument &){++invalid_attitude_;}
      });
    }
    // Subscribe configured anchors immediately; no 1 s discovery delay at startup.
    for(const auto &[id,a]:anchors_){
      range_subs_.push_back(create_subscription<sensor_msgs::msg::Range>("/uwb/anchor_"+std::to_string(id)+"/range",rclcpp::QoS(100),
        [this,id](sensor_msgs::msg::Range::ConstSharedPtr r){on_range(id,rclcpp::Time(r->header.stamp).nanoseconds(),r->range);}));
    }
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("~/odom",10);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",10);
    pub_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{publish();});
    stats_=create_wall_timer(std::chrono::milliseconds(100),[this]{status();});
    RCLCPP_INFO(get_logger(),"%zu anchors, imu=%s, attitude=%s, confirmed=%d, na=%.4f ng=%.5f",anchors_.size(),mode_.c_str(),params_.attitude_init.c_str(),confirmed_,params_.na,params_.ng);
  }
 private:
  bool eskf()const{return mode_!="none";}
  static Eigen::Isometry3d link_pose(const urdf::Model &m,const std::string &name){
    auto link=m.getLink(name);if(!link)throw std::runtime_error("URDF missing link "+name);
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    for(;link->parent_joint;link=link->getParent()){
      const auto &o=link->parent_joint->parent_to_joint_origin_transform;
      T=Eigen::Translation3d(o.position.x,o.position.y,o.position.z)*Eigen::Quaterniond(o.rotation.w,o.rotation.x,o.rotation.y,o.rotation.z)*T;
    }return T;
  }
  static std::string quote(const std::string &value){
    std::ostringstream out;out<<'"';for(unsigned char c:value){
      if(c=='"'||c=='\\')out<<'\\'<<c;else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;else out<<c;
    }out<<'"';return out.str();
  }
  // Subtract integer nanoseconds before converting to double. Unix seconds lose
  // enough precision to perturb the 20 ms bin/rate boundaries by hundreds of ns.
  double core_time(int64_t stamp){if(!time_origin_ns_)time_origin_ns_=stamp;return double(stamp-*time_origin_ns_)*1e-9;}
  bool fresh_stamp(int64_t stamp){const double age=double(now().nanoseconds()-stamp)*1e-9;return stamp>=0&&age>=-.1&&age<input_age_;}
  void reset_position(const std::string &reason){core_->reset_position();cv_ready_=false;cv_latest_.clear();cv_rejects_=0;last_range_good_=-1;reason_=reason;}
  void on_imu(int64_t stamp,const V3 &a,const V3 &w){
    if(!fresh_stamp(stamp)||!a.allFinite()||!w.allFinite()){++invalid_imu_;return;}
    const double t=core_time(stamp);if(t<=last_imu_stamp_){++invalid_imu_;return;}
    last_imu_stamp_=t;last_imu_rx_=now().seconds();last_w_=w;++n_imu_;
    const auto gaps=core_->counts.imu_gap_resets;core_->on_imu(t,a,w);
    if(core_->counts.imu_gap_resets!=gaps){reason_="IMU_GAP";last_range_good_=-1;}
    if(core_->initialized && (!core_->filter.p.allFinite()||!core_->filter.P.allFinite())){
      core_->reset_attitude(false);last_range_good_=-1;reason_="NONFINITE_STATE";
    }
  }
  void on_range(int id,int64_t stamp,double range){
    if(!fresh_stamp(stamp)||!std::isfinite(range)||range<=0||range>100){++invalid_range_;return;}
    const double t=core_time(stamp);if(last_stamp_.count(id)&&t<=last_stamp_[id]){++invalid_range_;return;}
    last_stamp_[id]=t;last_range_rx_=now().seconds();++n_range_;
    if(!confirmed_)return;
    if(!ignore_bias_)if(const auto b=bias_.find(id);b!=bias_.end())range-=b->second;
    if(range<=0){++invalid_range_;return;}
    if(!eskf()){on_cv_range(id,t,range);return;}
    if(core_->attitude_ready && (last_imu_rx_<0||now().seconds()-last_imu_rx_>imu_timeout_||std::abs(t-last_imu_stamp_)>imu_timeout_))return;
    const auto inits=core_->counts.initializations,reinits=core_->counts.reinits;
    core_->on_range(id,t,range);
    if(core_->counts.initializations!=inits || (core_->last_update.valid && core_->last_update.accepted)){
      last_range_good_=now().seconds();reason_="OK";
    }else if(core_->counts.reinits!=reinits){last_range_good_=-1;reason_="RANGE_GATE_RESET";}
  }
  // Retain the pre-existing UWB-only CV mode; inertial state-machine logic lives in Core.
  void on_cv_range(int id,double t,double range){
    if(!cv_ready_){
      cv_latest_[id]={t,range};std::vector<V3>A;std::vector<double>r;V3 center=V3::Zero();
      for(const auto &[aid,tr]:cv_latest_)if(t-tr.first>=0 && t-tr.first<params_.init_window_s){A.push_back(anchors_.at(aid));r.push_back(tr.second);center+=anchors_.at(aid);}
      if(A.size()<size_t(params_.init_min_anchors)||!Core::geometry_ok(A))return;
      const V3 p=uwb::gauss_newton(A,r,center/A.size());double ss=0;
      for(size_t i=0;i<A.size();++i)ss+=std::pow((p-A[i]).norm()-r[i],2);
      V3 lo=anchors_.begin()->second,hi=lo;for(const auto &[aid,a]:anchors_){lo=lo.cwiseMin(a);hi=hi.cwiseMax(a);}
      if(!p.allFinite()||std::sqrt(ss/A.size())>3*params_.sig_r||(p.array()<lo.array()-2).any()||(p.array()>hi.array()+2).any()){++cv_invalid_init_;return;}
      cv_.init(p);cv_ready_=true;cv_t_=t;++cv_initializations_;last_range_good_=now().seconds();reason_="OK";
    }else{
      if(t>cv_t_){cv_.predict(t-cv_t_,params_.q_cv);cv_t_=t;}double nis=0;
      if(cv_.update(anchors_.at(id),range,params_,nis)){++cv_accepted_;cv_rejects_=0;last_range_good_=now().seconds();}
      else if(++cv_rejects_>params_.reinit_after_rejects)reset_position("RANGE_GATE_RESET");
    }
  }
  void publish(){
    if(!confirmed_)return;const double current=now().seconds();
    if(eskf() && core_->raw_previous && current-last_imu_rx_>imu_timeout_){
      core_->reset_attitude();last_range_good_=-1;reason_="IMU_TIMEOUT";return;
    }
    if(!(eskf()?core_->initialized:cv_ready_))return;
    if(current-last_range_good_>range_timeout_){reset_position("RANGE_TIMEOUT");return;}
    const double stamp=eskf()?core_->sample_time:cv_t_;if(stamp<0)return;
    const auto &f=core_->filter;
    const V3 p=eskf()?f.p:V3(cv_.x.head<3>()),v=eskf()?V3(f.R.transpose()*f.v):V3(cv_.x.tail<3>());
    const Eigen::Quaterniond q(eskf()?f.R:M3::Identity());
    if(!p.allFinite()||!v.allFinite()||!q.coeffs().allFinite()){core_->reset_attitude(false);cv_ready_=false;reason_="NONFINITE_STATE";return;}
    nav_msgs::msg::Odometry odom;odom.header.stamp=rclcpp::Time(*time_origin_ns_+int64_t(std::llround(stamp*1e9)));odom.header.frame_id=frame_;odom.child_frame_id=body_;
    odom.pose.pose.position.x=p.x();odom.pose.pose.position.y=p.y();odom.pose.pose.position.z=p.z();
    auto &orientation=odom.pose.pose.orientation;orientation.w=q.w();orientation.x=q.x();orientation.y=q.y();orientation.z=q.z();
    odom.twist.twist.linear.x=v.x();odom.twist.twist.linear.y=v.y();odom.twist.twist.linear.z=v.z();
    const V3 w=eskf()?V3(last_w_-f.bg):V3::Zero();
    odom.twist.twist.angular.x=w.x();odom.twist.twist.angular.y=w.y();odom.twist.twist.angular.z=w.z();
    const M3 vcov=eskf()?M3(f.R.transpose()*f.P.block<3,3>(3,3)*f.R):M3(cv_.P.block<3,3>(3,3));
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
      odom.pose.covariance[i*6+j]=eskf()?f.P(i,j):cv_.P(i,j);
      odom.pose.covariance[(i+3)*6+j+3]=eskf()?f.P(6+i,6+j):(i==j?1e6:0);
      odom.twist.covariance[i*6+j]=vcov(i,j);
      if(eskf()){odom.pose.covariance[i*6+j+3]=f.P(i,6+j);odom.pose.covariance[(i+3)*6+j]=f.P(6+i,j);}
    }odom_pub_->publish(odom);++n_odom_;
  }
  void status(){
    const std::string state=!confirmed_?"WAIT_CONFIGURATION":eskf()&&last_imu_rx_<0?"WAIT_IMU":eskf()&&!core_->attitude_ready?
      (use_attitude_?"WAIT_PX4_ATTITUDE":"WAIT_STATIC_IMU"):!(eskf()?core_->initialized:cv_ready_)?"WAIT_RANGES":"RUNNING";
    const auto &c=core_->counts;const double current=now().seconds();int recent=0;
    const double current_core=time_origin_ns_?double(now().nanoseconds()-*time_origin_ns_)*1e-9:0.;
    for(const auto &[id,t]:last_stamp_)if(current_core-t<params_.init_window_s)++recent;
    std::ostringstream s;s<<std::setprecision(12)<<"{\"state\":"<<quote(state)<<",\"reason\":"<<quote(reason_)
      <<",\"configuration_confirmed\":"<<(confirmed_?"true":"false")<<",\"config_sha256\":"<<quote(config_sha_)
      <<",\"imu\":"<<quote(mode_)<<",\"rig_file\":"<<quote(rig_)<<",\"frame_id\":"<<quote(frame_)<<",\"body_frame\":"<<quote(body_)
      <<",\"configured_anchors\":"<<anchors_.size()<<",\"ignore_rig_bias\":"<<(ignore_bias_?"true":"false")
      <<",\"lever_flu_m\":["<<lever_.x()<<','<<lever_.y()<<','<<lever_.z()<<"],\"required_anchors\":"<<params_.init_min_anchors<<",\"recent_anchors\":"<<recent
      <<",\"imu_samples\":"<<n_imu_<<",\"range_samples\":"<<n_range_<<",\"accepted_ranges\":"<<(eskf()?c.accepted_ranges:cv_accepted_)
      <<",\"odom_messages\":"<<n_odom_<<",\"initializations\":"<<(eskf()?c.initializations:cv_initializations_)<<",\"reinits\":"<<c.reinits
      <<",\"invalid_imu\":"<<invalid_imu_<<",\"invalid_ranges\":"<<invalid_range_<<",\"invalid_initializations\":"<<(c.invalid_initializations+cv_invalid_init_)
      <<",\"invalid_attitudes\":"<<invalid_attitude_<<",\"attitude_mode\":"<<quote(params_.attitude_init)<<",\"rig_yaw_offset_deg\":";
    if(rig_yaw_offset_)s<<*rig_yaw_offset_*180/M_PI;else s<<"null";
    s<<",\"attitude_accepted\":"<<c.accepted_attitudes<<",\"attitude_rejected\":"<<c.rejected_attitudes
      <<",\"skipped_ranges\":"<<c.skipped_ranges<<",\"imu_gap_resets\":"<<c.imu_gap_resets<<",\"attitude_inits\":"<<c.attitude_inits
      <<",\"imu_age_s\":"<<(last_imu_rx_<0?-1:current-last_imu_rx_)<<",\"range_age_s\":"<<(last_range_rx_<0?-1:current-last_range_rx_)
      <<",\"parameters\":{\"sig_r\":"<<params_.sig_r<<",\"q_cv\":"<<params_.q_cv<<",\"gate\":"<<params_.gate<<",\"k_pos\":"<<params_.k_pos<<",\"k_neg\":"<<params_.k_neg
      <<",\"na\":"<<params_.na<<",\"ng\":"<<params_.ng<<",\"nba\":"<<params_.nba<<",\"nbg\":"<<params_.nbg
      <<",\"min_anchor_interval_s\":"<<params_.min_anchor_interval_s<<",\"imu_average_s\":"<<params_.imu_average_s<<",\"use_px4_attitude\":"<<use_attitude_
      <<",\"reinit_keep_attitude\":"<<(params_.reinit_keep_attitude?"true":"false")<<",\"reinit_after_rejects\":"<<params_.reinit_after_rejects
      <<",\"att_sig_tilt_deg\":"<<params_.att_sig_deg.x()<<",\"att_sig_yaw_deg\":"<<params_.att_sig_deg.z()<<",\"att_gate\":"<<params_.att_gate
      <<",\"att_min_interval_s\":"<<params_.att_min_interval_s<<",\"init_static_s\":"<<params_.init_static_s<<",\"init_window_s\":"<<params_.init_window_s
      <<",\"min_init_anchors\":"<<params_.init_min_anchors<<",\"imu_timeout_s\":"<<imu_timeout_<<",\"imu_gap_reset_s\":"<<params_.imu_gap_reset_s
      <<",\"range_timeout_s\":"<<range_timeout_<<",\"max_input_age_s\":"<<input_age_<<",\"static_acc_tol\":"<<params_.static_acc_tol<<",\"static_gyro_tol\":"<<params_.static_gyro_tol<<"}}";
    std_msgs::msg::String message;message.data=s.str();status_pub_->publish(message);
  }
  Core::Params params_;std::unique_ptr<Core>core_;uwb::CV cv_;
  std::map<int,V3>anchors_;std::map<int,double>bias_,last_stamp_;std::map<int,std::pair<double,double>>cv_latest_;
  std::string rig_,mode_,frame_,body_,config_sha_,attitude_topic_,reason_="STARTING";
  bool confirmed_=false,ignore_bias_=false,cv_ready_=false;int use_attitude_=1,cv_rejects_=0;
  double yaw0_=0,imu_timeout_=.2,range_timeout_=.5,input_age_=.25,last_imu_stamp_=-1,last_attitude_stamp_=-1;
  double last_imu_rx_=-1,last_range_rx_=-1,last_range_good_=-1,cv_t_=-1;
  std::optional<double>rig_yaw_offset_;V3 lever_=V3::Zero(),last_w_=V3::Zero();M3 imu_R_=M3::Identity();
  std::optional<int64_t>time_origin_ns_;
  uint64_t n_imu_=0,n_range_=0,n_odom_=0,invalid_imu_=0,invalid_range_=0,invalid_attitude_=0,cv_initializations_=0,cv_accepted_=0,cv_invalid_init_=0;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>range_subs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#ifdef UWB_ESKF_WITH_PX4_MSGS
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_sub_;
#endif
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr att_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_,stats_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<UwbEskfNode>());rclcpp::shutdown();}
