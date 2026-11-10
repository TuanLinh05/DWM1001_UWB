// Standalone: g++ -std=c++17 -O2 -I../include -I/usr/include/eigen3 estimator_parity.cpp -o estimator_parity
// The same runner is exercised as a gtest under colcon.
#include "uwb_eskf/estimator.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace fs=std::filesystem;
using uwb::V3;using uwb::M3;using Core=uwb::Estimator;

static std::vector<std::string> cells(const std::string &line){
  std::vector<std::string> out;std::istringstream input(line);std::string s;
  while(std::getline(input,s,',')){if(!s.empty()&&s.back()=='\r')s.pop_back();out.push_back(s);}return out;
}
static std::vector<double> numbers(const std::string &line){
  std::vector<double> out;for(const auto &s:cells(line))out.push_back(std::stod(s));return out;
}
static Core load_core(const fs::path &dir){
  Core::Params p;V3 lever=V3::Zero();double yaw0=0;
  std::ifstream config(dir/"config.csv");if(!config)throw std::runtime_error("missing config.csv");
  for(std::string line;std::getline(config,line);){
    const auto v=cells(line);if(v.size()<2)continue;const auto &k=v[0];
    if(k=="attitude_init"){p.attitude_init=v[1];continue;}
    const double x=std::stod(v[1]);
#define VALUE(name) if(k==#name){p.name=x;continue;}
    VALUE(sig_r) VALUE(gate) VALUE(k_pos) VALUE(k_neg) VALUE(na) VALUE(ng) VALUE(nba) VALUE(nbg)
    VALUE(init_static_s) VALUE(init_window_s) VALUE(init_min_anchors) VALUE(reinit_after_rejects)
    VALUE(sig_p0) VALUE(sig_v0) VALUE(sig_ba0) VALUE(sig_bg0_deg) VALUE(min_anchor_interval_s)
    VALUE(reinit_keep_attitude) VALUE(att_gate) VALUE(att_min_interval_s) VALUE(imu_average_s)
    VALUE(static_check) VALUE(static_acc_tol) VALUE(static_gyro_tol) VALUE(imu_gap_reset_s)
#undef VALUE
    if(k=="lever")lever=V3(x,std::stod(v[2]),std::stod(v[3]));
    else if(k=="yaw0")yaw0=x;
    else if(k=="sig_th0_deg")p.sig_th0_deg=V3(x,std::stod(v[2]),std::stod(v[3]));
    else if(k=="att_sig_deg")p.att_sig_deg=V3(x,std::stod(v[2]),std::stod(v[3]));
    else throw std::runtime_error("unknown core parameter: "+k);
  }
  std::map<int,V3> anchors;std::ifstream af(dir/"anchors.csv");
  for(std::string line;std::getline(af,line);){auto v=numbers(line);anchors[int(v[0])]=V3(v[1],v[2],v[3]);}
  return Core(p,anchors,lever,yaw0);
}
static void write_state(std::ostream &out,const Core &c){
  const auto &f=c.filter;out<<c.sample_time;
  for(const auto &v:{f.p,f.v})for(int i=0;i<3;++i)out<<','<<v[i];
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)out<<','<<f.R(i,j);
  for(const auto &v:{f.ba,f.bg})for(int i=0;i<3;++i)out<<','<<v[i];
  for(int i=0;i<15;++i)for(int j=0;j<15;++j)out<<','<<f.P(i,j);
  out<<'\n';
}
static void run_bundle(const fs::path &dir,const fs::path &out){
  Core core=load_core(dir);fs::create_directories(out);
  std::ifstream input(dir/"events.csv");if(!input)throw std::runtime_error("missing events.csv");
  std::ofstream states(out/"cpp_states.csv"),updates(out/"cpp_updates.csv"),summary(out/"cpp_summary.json");
  states<<std::setprecision(17);updates<<std::setprecision(17);summary<<std::setprecision(17);
  for(std::string line;std::getline(input,line);){
    auto v=numbers(line);const int kind=int(v[0]);const double t=v[1];const auto serial=core.output_serial;
    if(kind==0)core.on_imu(t,V3(v[3],v[4],v[5]),V3(v[6],v[7],v[8]));
    else if(kind==1)core.on_range(int(v[2]),t,v[3]);
    else if(kind==2){M3 R;for(int i=0;i<3;++i)for(int j=0;j<3;++j)R(i,j)=v[3+i*3+j];core.on_attitude(t,R);}
    else throw std::runtime_error("unknown event kind");
    if(core.output_serial!=serial)write_state(states,core);
    const auto &u=core.last_update;
    if(u.valid)updates<<u.kind<<','<<u.t<<','<<u.id<<','<<int(u.accepted)<<','<<u.nis<<','<<u.innovation<<'\n';
  }
  const auto &c=core.counts;
  summary<<"{\"reinits\":"<<c.reinits<<",\"skipped_ranges\":"<<c.skipped_ranges<<",\"imu_gap_resets\":"<<c.imu_gap_resets
         <<",\"attitude_inits\":"<<c.attitude_inits<<",\"initializations\":"<<c.initializations<<",\"offered_ranges\":"<<c.offered_ranges
         <<",\"accepted_ranges\":"<<c.accepted_ranges<<",\"accepted_attitudes\":"<<c.accepted_attitudes<<",\"rejected_attitudes\":"<<c.rejected_attitudes
         <<",\"output_rows\":"<<core.output_serial<<",\"init_time\":";
  if(core.init_time)summary<<*core.init_time;else summary<<"null";summary<<"}\n";
}
static void run_frames(const fs::path &input_path,const fs::path &output_path){
  std::ifstream input(input_path);std::ofstream out(output_path);out<<std::setprecision(17);
  for(std::string line;std::getline(input,line);){
    const auto v=numbers(line);const Eigen::Vector4d q(v[0],v[1],v[2],v[3]);const M3 R=uwb::px4_attitude_to_rig(q,v[4]);
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)out<<R(i,j)<<',';
    out<<uwb::px4_yaw_ned(q)<<','<<uwb::rig_yaw_offset_from_yaw0(q,v[5])<<'\n';
  }
}

#ifdef ESKF_PARITY_GTEST
#include <gtest/gtest.h>
TEST(EstimatorParity, GoldenFullStatesAndEveryUpdate){
  const fs::path input(PARITY_DIR);
  const auto output=fs::temp_directory_path()/("uwb_eskf_parity_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_NO_THROW(run_bundle(input,output));
  for(const auto &kind:{"states","updates"}){
    std::ifstream expected(input/(std::string("python_")+kind+".csv")),actual(output/(std::string("cpp_")+kind+".csv"));
    ASSERT_TRUE(expected.good());ASSERT_TRUE(actual.good());std::string a,b;size_t rows=0;
    while(std::getline(expected,a)){
      ASSERT_TRUE(bool(std::getline(actual,b)));const auto x=numbers(a),y=numbers(b);ASSERT_EQ(x.size(),y.size());
      for(size_t i=0;i<x.size();++i){
        if(std::isnan(x[i]))EXPECT_TRUE(std::isnan(y[i]));
        else if(std::string(kind)=="updates" && (i==0||i==2||i==3))EXPECT_EQ(x[i],y[i]);
        else EXPECT_NEAR(x[i],y[i],1e-6*std::max(1.,std::abs(x[i])))<<"row="<<rows<<" col="<<i;
      }++rows;
    }EXPECT_FALSE(bool(std::getline(actual,b)));EXPECT_GT(rows,0u);
  }
  fs::remove_all(output);
}
TEST(EstimatorFrames, HamiltonAndYawOffset){
  const Eigen::Vector4d q(1,0,0,0);const M3 R=uwb::px4_attitude_to_rig(q,.7);
  EXPECT_NEAR(R.determinant(),1.,1e-12);EXPECT_NEAR(std::atan2(R(1,0),R(0,0)),.7,1e-12);
  EXPECT_THROW(uwb::quat_to_matrix(Eigen::Vector4d::Zero()),std::invalid_argument);
  EXPECT_NEAR((uwb::so3_exp(uwb::log_so3(uwb::so3_exp(V3(M_PI,0,0))))-uwb::so3_exp(V3(M_PI,0,0))).norm(),0.,1e-12);
}
TEST(EstimatorAttitude, GateLeavesAllStateAndCovarianceUnchanged){
  uwb::ESKF f;f.init(V3::Zero(),M3::Identity());f.P*=.001;
  const auto P=f.P;const auto R=f.R;double nis=0;
  EXPECT_FALSE(f.attitude_update(uwb::so3_exp(V3(0,0,M_PI/2)),V3(2,2,5),25,nis));
  EXPECT_GT(nis,25);EXPECT_EQ((f.P-P).norm(),0);EXPECT_EQ((f.R-R).norm(),0);
}
TEST(EstimatorDegenerate, CoincidentSeedAndTagAnchorAreHandledLikePython){
  std::vector<V3> anchors={V3::Zero(),V3(1,0,0),V3(0,1,0),V3(0,0,1)};
  const auto p=uwb::gauss_newton(anchors,{1,1,1,1},V3::Zero());
  EXPECT_FALSE(p.allFinite());
  Core::Params params;Core core(params,{{1,V3::Zero()}},V3::Zero(),0.);
  core.filter.init(V3::Zero(),M3::Identity());core.initialized=core.attitude_ready=true;
  const auto P=core.filter.P;core.on_range(1,1.,1.);
  EXPECT_TRUE(core.last_update.valid);EXPECT_FALSE(core.last_update.accepted);
  EXPECT_TRUE(std::isnan(core.last_update.innovation));EXPECT_EQ((core.filter.P-P).norm(),0.);
}
#else
int main(int argc,char **argv){
  try{
    if(argc==4 && std::string(argv[1])=="--frames")run_frames(argv[2],argv[3]);
    else if(argc==3)run_bundle(argv[1],argv[2]);
    else{std::cerr<<"Usage: estimator_parity INPUT_BUNDLE OUTPUT_DIR | --frames INPUT OUTPUT\n";return 2;}
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}return 0;
}
#endif
