// UWB TWR filter, port 1-1 tu tools/uwb_eskf.py (ban tham chieu Python; chay ./uwb_eskf.py --selftest).
// Frame: world = rig (z len), body FLU. Range: h = |p + R l - a|, update vo huong + Huber bat doi xung (NLOS lam range dai).
#pragma once
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace uwb {

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

struct Params {
  double sig_r = 0.15;            // [m] std residual that cua rig (CAL_MISSING), xem uwb_eskf.py
  double gate = 25.0;             // gate cung 5 sigma
  double k_pos = 1.5, k_neg = 3.0;  // Huber: range dai (NLOS) giam trong so som hon
  double q_cv = 1.0;              // [m^2/s^3]
  double na = 175e-6 * 9.81, ng = 0.014 * M_PI / 180;  // BMI088 [m/s^2/sqrtHz], [rad/s/sqrtHz]
  double nba = 3e-3, nbg = 1e-4;                       // bias random walk
};

inline M3 skew(const V3 &v) {
  M3 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

inline M3 so3_exp(const V3 &phi) {
  const double th = phi.norm();
  if (th < 1e-9) return M3::Identity() + skew(phi);
  const M3 k = skew(phi / th);
  return M3::Identity() + std::sin(th) * k + (1 - std::cos(th)) * k * k;
}

// Update vo huong robust. false = bi gate. dx tra qua tham so.
template <int N>
bool range_update(Eigen::Matrix<double, N, N> &P, const Eigen::Matrix<double, N, 1> &H, double y, double r_var,
                  const Params &pr, Eigen::Matrix<double, N, 1> &dx, double &nis) {
  const Eigen::Matrix<double, N, 1> PH = P * H;
  const double HPH = H.dot(PH);
  double S = HPH + r_var;
  nis = y * y / S;
  if (nis > pr.gate) return false;
  const double z = std::sqrt(nis), k = y > 0 ? pr.k_pos : pr.k_neg;
  if (z > k) {  // IRLS 1 buoc: noi R sao cho S_eff = S z / k
    r_var = S * z / k - HPH;
    S = HPH + r_var;
  }
  const Eigen::Matrix<double, N, 1> K = PH / S;
  dx = K * y;
  const Eigen::Matrix<double, N, N> IKH = Eigen::Matrix<double, N, N>::Identity() - K * H.transpose();
  P = IKH * P * IKH.transpose() + r_var * K * K.transpose();
  return true;
}

struct CV {
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Matrix<double, 6, 6> P;

  void init(const V3 &p0, double sig_p = 0.3, double sig_v = 1.0) {
    x << p0, V3::Zero();
    P.setZero();
    P.diagonal() << V3::Constant(sig_p * sig_p), V3::Constant(sig_v * sig_v);
  }
  void predict(double dt, double q) {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity(), Q;
    F.block<3, 3>(0, 3) = dt * M3::Identity();
    const M3 I = M3::Identity();
    Q << dt * dt * dt / 3 * I, dt * dt / 2 * I, dt * dt / 2 * I, dt * I;
    x = F * x;
    P = F * P * F.transpose() + q * Q;
  }
  bool update(const V3 &a, double r, const Params &pr, double &nis) {
    const V3 d = x.head<3>() - a;
    const double h = d.norm();
    if (!std::isfinite(h) || h < 1e-6) return false;
    Eigen::Matrix<double, 6, 1> H = Eigen::Matrix<double, 6, 1>::Zero(), dx;
    H.head<3>() = d / h;
    if (!range_update<6>(P, H, r - h, pr.sig_r * pr.sig_r, pr, dx, nis)) return false;
    x += dx;
    return true;
  }
};

struct ESKF {
  V3 p, v, ba, bg;
  M3 R;
  Eigen::Matrix<double, 15, 15> P;

  void init(const V3 &p0, const M3 &R0, double sig_p = 0.3, double sig_v = 0.5,
            const V3 &sig_th = V3(5, 5, 30) * M_PI / 180, double sig_ba = 0.2, double sig_bg = M_PI / 180) {
    p = p0, v.setZero(), R = R0, ba.setZero(), bg.setZero();
    P.setZero();
    P.diagonal() << V3::Constant(sig_p * sig_p), V3::Constant(sig_v * sig_v), sig_th.cwiseAbs2(),
        V3::Constant(sig_ba * sig_ba), V3::Constant(sig_bg * sig_bg);
  }
  void predict(const V3 &a_m, const V3 &w_m, double dt, const Params &pr) {
    const V3 a = a_m - ba, w = w_m - bg, acc = R * a + V3(0, 0, -9.81);
    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3, 3>(0, 3) = dt * M3::Identity();
    F.block<3, 3>(3, 6) = -R * skew(a) * dt;
    F.block<3, 3>(3, 9) = -R * dt;
    F.block<3, 3>(6, 6) = so3_exp(-w * dt);
    F.block<3, 3>(6, 12) = -M3::Identity() * dt;
    Eigen::Matrix<double, 15, 1> q;
    q << V3::Zero(), V3::Constant(pr.na * pr.na * dt), V3::Constant(pr.ng * pr.ng * dt), V3::Constant(pr.nba * pr.nba * dt),
        V3::Constant(pr.nbg * pr.nbg * dt);
    p = p + v * dt + 0.5 * acc * dt * dt;
    v = v + acc * dt;
    R = R * so3_exp(w * dt);
    P = F * P * F.transpose();
    P.diagonal() += q;
  }
  bool update(const V3 &a, double r, const V3 &lever, const Params &pr, double &nis) {
    const V3 d = p + R * lever - a;
    const double h = d.norm();
    if (!std::isfinite(h) || h < 1e-6) return false;
    const V3 u = d / h;
    Eigen::Matrix<double, 15, 1> H = Eigen::Matrix<double, 15, 1>::Zero(), dx;
    H.head<3>() = u;
    H.segment<3>(6) = (u.transpose() * (-R * skew(lever))).transpose();
    if (!range_update<15>(P, H, r - h, pr.sig_r * pr.sig_r, pr, dx, nis)) return false;
    p += dx.segment<3>(0), v += dx.segment<3>(3), R = R * so3_exp(dx.segment<3>(6));  // inject, reset G ~ I
    ba += dx.segment<3>(9), bg += dx.segment<3>(12);
    return true;
  }
};

// Nghiem tinh 1 epoch (init / re-init sau khi lac).
inline V3 gauss_newton(const std::vector<V3> &A, const std::vector<double> &r, V3 p, int iters = 10) {
  const int n = static_cast<int>(A.size());
  for (int it = 0; it < iters; ++it) {
    Eigen::MatrixXd J(n, 3);
    Eigen::VectorXd e(n);
    for (int i = 0; i < n; ++i) {
      const V3 d = p - A[i];
      J.row(i) = d.transpose() / d.norm();
      e(i) = d.norm() - r[i];
    }
    p -= J.colPivHouseholderQr().solve(e);
  }
  return p;
}

// Roll/pitch tu gia toc dung yen (FLU, z len: dung yen doc +g), yaw cho truoc.
inline M3 attitude_from_accel(const V3 &f, double yaw) {
  const double roll = std::atan2(f.y(), f.z()), pitch = std::atan2(-f.x(), std::hypot(f.y(), f.z()));
  return (Eigen::AngleAxisd(yaw, V3::UnitZ()) * Eigen::AngleAxisd(pitch, V3::UnitY()) *
          Eigen::AngleAxisd(roll, V3::UnitX())).toRotationMatrix();
}

// "an3_p, x, y, z" -> {3: (x,y,z)}; bo dong '#'.
inline std::map<int, V3> load_rig(const std::string &path) {
  std::map<int, V3> a;
  std::ifstream f(path);
  for (std::string line; std::getline(f, line);) {
    if (line.empty() || line[0] == '#' || line.rfind("an", 0) != 0) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream s(line);
    std::string key;
    V3 p;
    if (s >> key >> p.x() >> p.y() >> p.z()) a[std::stoi(key.substr(2, key.find('_') - 2))] = p;
  }
  return a;
}

// "an3_bias, b" -> {3: b} (meas = true + b, m); anchor khong co dong nao -> 0. Dong '#' bi bo nhu load_rig.
inline std::map<int, double> load_bias(const std::string &path) {
  std::map<int, double> b;
  std::ifstream f(path);
  for (std::string line; std::getline(f, line);) {
    if (line.rfind("an", 0) != 0 || line.find("_bias") == std::string::npos) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream s(line);
    std::string key;
    double v;
    if (s >> key >> v) b[std::stoi(key.substr(2, key.find('_') - 2))] = v;
  }
  return b;
}

}  // namespace uwb
