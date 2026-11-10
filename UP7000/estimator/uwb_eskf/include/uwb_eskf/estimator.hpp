// Chronological, ROS-free ESKF event core. Python reference: eskf_replay.EstimatorCore.
#pragma once
#include "filter.hpp"
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace uwb {

inline bool valid_rotation(const M3 &R){
  return R.allFinite() && (R.transpose()*R-M3::Identity()).cwiseAbs().maxCoeff()<=1e-6 && R.determinant()>0;
}
inline M3 quat_to_matrix(const Eigen::Vector4d &q){
  const double n=q.norm();
  if(!q.allFinite() || !(n>.5 && n<2))throw std::invalid_argument("invalid Hamilton quaternion");
  return Eigen::Quaterniond(q[0]/n,q[1]/n,q[2]/n,q[3]/n).toRotationMatrix();
}
inline double px4_yaw_ned(const Eigen::Vector4d &q){const M3 R=quat_to_matrix(q);return std::atan2(R(1,0),R(0,0));}
inline M3 px4_attitude_to_rig(const Eigen::Vector4d &q,double offset){
  if(!std::isfinite(offset))throw std::invalid_argument("invalid rig yaw offset");
  M3 T;T<<0,1,0,1,0,0,0,0,-1;
  return Eigen::AngleAxisd(offset-M_PI/2,V3::UnitZ()).toRotationMatrix()*T*quat_to_matrix(q)*V3(1,-1,-1).asDiagonal();
}
inline double rig_yaw_offset_from_yaw0(const Eigen::Vector4d &q,double yaw0){
  if(!std::isfinite(yaw0))throw std::invalid_argument("invalid initial yaw");return yaw0+px4_yaw_ned(q);
}

class Estimator {
 public:
  struct Params : uwb::Params {
    double init_static_s=1.,init_window_s=.05;
    int init_min_anchors=6,reinit_after_rejects=200;
    double sig_p0=.3,sig_v0=.5;
    V3 sig_th0_deg=V3(5,5,30);
    double sig_ba0=.2,sig_bg0_deg=1.;
    double min_anchor_interval_s=0.;bool reinit_keep_attitude=false;
    V3 att_sig_deg=V3(2,2,5);double att_gate=25.,att_min_interval_s=.02;
    double imu_average_s=0.;std::string attitude_init="static";
    bool static_check=false;double static_acc_tol=.8,static_gyro_tol=.15;
    double imu_gap_reset_s=std::numeric_limits<double>::infinity();
  };
  struct Counters {
    uint64_t reinits=0,skipped_ranges=0,imu_gap_resets=0,attitude_inits=0,initializations=0;
    uint64_t offered_ranges=0,accepted_ranges=0,accepted_attitudes=0,rejected_attitudes=0,invalid_initializations=0;
  } counts;
  struct Update {
    bool valid=false,accepted=false;int kind=0,id=0;double t=0,nis=0,innovation=std::numeric_limits<double>::quiet_NaN();
  } last_update;
  Params params;std::map<int,V3> anchors;V3 lever;double yaw0;
  ESKF filter;bool initialized=false,attitude_ready=false;
  std::optional<double> init_time,raw_previous;
  double sample_time=-1.;uint64_t output_serial=0;

  Estimator(const Params &pr,const std::map<int,V3> &a,const V3 &l,double yaw):params(pr),anchors(a),lever(l),yaw0(yaw){
    if(!lever.allFinite() || !std::isfinite(yaw0))throw std::invalid_argument("invalid lever/yaw");
    for(const auto &[id,p]:anchors)if(!p.allFinite())throw std::invalid_argument("invalid anchor");
    if(!(std::isfinite(pr.imu_average_s)&&pr.imu_average_s>=0) || (pr.attitude_init!="static"&&pr.attitude_init!="external")
       || std::isnan(pr.imu_gap_reset_s)||pr.imu_gap_reset_s<=0)throw std::invalid_argument("invalid IMU/init parameters");
    if(!std::isfinite(pr.min_anchor_interval_s)||pr.min_anchor_interval_s<0 || !std::isfinite(pr.att_min_interval_s)||pr.att_min_interval_s<0
       || !pr.att_sig_deg.allFinite() || pr.att_sig_deg.minCoeff()<=0 || !std::isfinite(pr.att_gate)||pr.att_gate<=0
       || !std::isfinite(pr.sig_r)||pr.sig_r<=0 || !std::isfinite(pr.gate)||pr.gate<=0
       || !std::isfinite(pr.k_pos)||pr.k_pos<=0 || !std::isfinite(pr.k_neg)||pr.k_neg<=0)
      throw std::invalid_argument("invalid measurement parameters");
    if(!std::isfinite(pr.static_acc_tol)||pr.static_acc_tol<0||!std::isfinite(pr.static_gyro_tol)||pr.static_gyro_tol<0)
      throw std::invalid_argument("invalid static tolerances");
    if(!anchors.empty()){lo_=hi_=anchors.begin()->second;for(const auto &[id,p]:anchors){lo_=lo_.cwiseMin(p);hi_=hi_.cwiseMax(p);}}
  }
  void reset_attitude(bool imu_gap=true){
    initialized=attitude_ready=false;carry_=false;R0_=M3::Identity();bg0_.setZero();raw_previous.reset();previous_imu_.reset();
    sample_time=-1.;clear_static();average_origin_.reset();average_bin_.reset();average_count_=0;average_t_=0;
    average_a_.setZero();average_w_.setZero();latest_.clear();rejected_run_=0;last_update={};
    if(imu_gap)++counts.imu_gap_resets;
    // external_R_ and rate-limit history survive; only a fresh attitude event can re-arm.
  }
  void reset_position(){
    if(initialized && params.reinit_keep_attitude){carry_=true;carry_R_=filter.R;carry_bg_=filter.bg;}
    initialized=false;latest_.clear();rejected_run_=0;
  }
  bool on_imu(double t,const V3 &a,const V3 &w){
    last_update={};if(!std::isfinite(t)||!a.allFinite()||!w.allFinite())return false;
    if(raw_previous && t-*raw_previous>params.imu_gap_reset_s)reset_attitude();
    raw_previous=t;const auto serial=output_serial;
    if(params.imu_average_s>0){
      if(!average_origin_)average_origin_=t;
      const int64_t bin=static_cast<int64_t>(std::floor((t-*average_origin_)/params.imu_average_s+1e-9));
      if(average_bin_ && bin!=*average_bin_){
        process_imu(average_t_/average_count_,average_a_/average_count_,average_w_/average_count_);
        average_count_=0;average_t_=0;average_a_.setZero();average_w_.setZero();
      }
      average_bin_=bin;average_t_+=t;average_a_+=a;average_w_+=w;++average_count_;
    }else process_imu(t,a,w);
    return output_serial!=serial;
  }
  void on_range(int id,double t,double r){
    last_update={};if(!anchors.count(id)||!std::isfinite(t)||!std::isfinite(r)||r<=.01)return;
    const auto last=last_range_.find(id);
    if(last!=last_range_.end() && t-last->second<params.min_anchor_interval_s-1e-9){++counts.skipped_ranges;return;}
    last_range_[id]=t;if(!attitude_ready)return;
    if(!initialized){
      latest_[id]={t,r};std::vector<V3>A;std::vector<double>rr;V3 center=V3::Zero();
      for(const auto &[aid,tr]:latest_)if(t-tr.first>=0 && t-tr.first<params.init_window_s){A.push_back(anchors.at(aid));rr.push_back(tr.second);center+=anchors.at(aid);}
      if(A.size()<static_cast<size_t>(std::max(4,std::min(params.init_min_anchors,int(anchors.size()))))||!geometry_ok(A))return;
      const V3 tag0=gauss_newton(A,rr,center/A.size());double ss=0;
      for(size_t i=0;i<A.size();++i)ss+=std::pow((tag0-A[i]).norm()-rr[i],2);
      const double rms=std::sqrt(ss/A.size());
      if(!tag0.allFinite()||rms>3*params.sig_r||(tag0.array()<lo_.array()-2).any()||(tag0.array()>hi_.array()+2).any()){++counts.invalid_initializations;return;}
      const M3 R=params.attitude_init=="external"?*external_R_:(carry_?carry_R_:R0_);
      const V3 bg=params.attitude_init=="external"?(carry_?carry_bg_:V3::Zero()):(carry_?carry_bg_:bg0_);
      filter.init(tag0-R*lever,R,params.sig_p0,params.sig_v0,params.sig_th0_deg*(M_PI/180),params.sig_ba0,params.sig_bg0_deg*M_PI/180);
      filter.bg=bg;initialized=true;if(!init_time)init_time=t;++counts.initializations;rejected_run_=0;
    }else{
      const V3 delta=filter.p+filter.R*lever-anchors.at(id);double nis=0;
      const double distance=delta.norm();
      const double innovation=(!std::isfinite(distance)||distance<1e-6)?std::numeric_limits<double>::quiet_NaN():r-distance;
      const bool accepted=filter.update(anchors.at(id),r,lever,params,nis);
      last_update={true,accepted,1,id,t,nis,innovation};++counts.offered_ranges;
      if(accepted){++counts.accepted_ranges;rejected_run_=0;}else ++rejected_run_;
      if(rejected_run_>params.reinit_after_rejects){reset_position();++counts.reinits;}
    }
  }
  void on_attitude(double t,const M3 &R){
    last_update={};if(!std::isfinite(t)||!valid_rotation(R))return;
    if(params.attitude_init=="external"){
      external_R_=R;
      if(!attitude_ready && raw_previous){attitude_ready=true;++counts.attitude_inits;}
    }
    if(initialized && t-last_attitude_>=params.att_min_interval_s-1e-9){
      last_attitude_=t;double nis=0;const bool accepted=filter.attitude_update(R,params.att_sig_deg,params.att_gate,nis);
      last_update={true,accepted,2,0,t,nis,std::numeric_limits<double>::quiet_NaN()};
      if(accepted)++counts.accepted_attitudes;else ++counts.rejected_attitudes;
    }
  }
  static bool geometry_ok(const std::vector<V3> &A){
    if(A.empty())return false;
    V3 lo=A[0],hi=A[0];for(const auto &p:A){lo=lo.cwiseMin(p);hi=hi.cwiseMax(p);}const double span=(hi-lo).maxCoeff();
    for(size_t i=0;i<A.size();++i)for(size_t j=i+1;j<A.size();++j)for(size_t k=j+1;k<A.size();++k)for(size_t l=k+1;l<A.size();++l){
      M3 d;d.col(0)=A[j]-A[i];d.col(1)=A[k]-A[i];d.col(2)=A[l]-A[i];if(std::abs(d.determinant())>std::max(1e-9,span*span*span*1e-7))return true;
    }return false;
  }
 private:
  V3 lo_=V3::Zero(),hi_=V3::Zero(),acc_sum_=V3::Zero(),gyro_sum_=V3::Zero(),bg0_=V3::Zero();M3 R0_=M3::Identity();
  std::optional<double>first_imu_,previous_imu_;int static_count_=0,rejected_run_=0;
  bool carry_=false;M3 carry_R_=M3::Identity();V3 carry_bg_=V3::Zero();std::optional<M3>external_R_;
  std::map<int,double>last_range_;double last_attitude_=-std::numeric_limits<double>::infinity();
  std::map<int,std::pair<double,double>>latest_;
  std::optional<double>average_origin_;std::optional<int64_t>average_bin_;
  int average_count_=0;double average_t_=0;V3 average_a_=V3::Zero(),average_w_=V3::Zero();
  void clear_static(){first_imu_.reset();acc_sum_.setZero();gyro_sum_.setZero();static_count_=0;}
  void process_imu(double t,const V3 &a,const V3 &w){
    if(!first_imu_)first_imu_=t;
    if(!attitude_ready){
      if(params.attitude_init=="static"){
        if(params.static_check && (std::abs(a.norm()-9.81)>params.static_acc_tol||w.norm()>params.static_gyro_tol))clear_static();
        else{acc_sum_+=a;gyro_sum_+=w;++static_count_;if(t-*first_imu_>=params.init_static_s){R0_=attitude_from_accel(acc_sum_/static_count_,yaw0);bg0_=gyro_sum_/static_count_;attitude_ready=true;++counts.attitude_inits;}}
      }
    }else if(initialized && previous_imu_ && t>*previous_imu_){filter.predict(a,w,t-*previous_imu_,params);++output_serial;}
    else if(carry_ && previous_imu_ && t>*previous_imu_)carry_R_=carry_R_*so3_exp((w-carry_bg_)*(t-*previous_imu_));
    previous_imu_=t;sample_time=t;
  }
};
}  // namespace uwb
