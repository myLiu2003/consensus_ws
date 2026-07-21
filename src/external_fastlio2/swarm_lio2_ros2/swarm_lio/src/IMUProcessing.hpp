#ifndef _IMU_PROCESSING_HPP
#define _IMU_PROCESSING_HPP

#define MAX_INI_COUNT (200)

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
//#include "MultiUAV.hpp"
#include "MultiUAV.h"


const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();
    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::msg::Imu::SharedPtr &lastimu);
    void set_extrinsic(const M3D &rot, const V3D &trans);
    void set_gyr_cov(const V3D &scaler);
    void set_acc_cov(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    void set_global_extrinsic_cov(const V3D &cov_global_extrinsic_rot, const V3D &cov_global_extrinsic_trans);
    void set_inten_threshold(const int &threshold);
    void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr orig_pcl_un_);

    V3D cov_acc, cov_gyr, cov_acc_scale, cov_gyr_scale;
    V3D cov_bias_gyr, cov_bias_acc;
    int lidar_type;
    V3D unbiased_gyr;
    M3D offset_R_L_I;
    V3D offset_T_L_I;
    V3D cov_global_extrinsic_rot, cov_global_extrinsic_trans;
    double IMU_mean_acc_norm = 0.0;
    bool imu_need_init_ = true;

private:
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
    void propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &orig_pcl_out);
    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    deque<sensor_msgs::msg::Imu::SharedPtr> v_imu_;
    vector<Pose6D> IMUpose;
    V3D mean_acc, mean_gyr, angvel_last, acc_s_last;
    double last_lidar_end_time_, time_last_scan;
    int init_iter_num = 1;
    bool b_first_frame_ = true;
};

ImuProcess::ImuProcess()
        : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    offset_R_L_I = M3D::Identity();
    offset_T_L_I = Zero3d;
    cov_global_extrinsic_rot = V3D(0.0001, 0.0001, 0.0001);
    cov_global_extrinsic_trans = V3D(0.0001, 0.0001, 0.0001);
    last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    printf("Reset ImuProcess\n");
    mean_acc = V3D(0, 0, -1.0);
    mean_gyr = V3D(0, 0, 0);
    angvel_last = Zero3d;
    imu_need_init_ = true;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset(new sensor_msgs::msg::Imu());
    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_gyr_cov(const V3D &scaler) { cov_gyr_scale = scaler; }
void ImuProcess::set_acc_cov(const V3D &scaler) { cov_acc_scale = scaler; }
void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }
void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }
void ImuProcess::set_extrinsic(const M3D &rot, const V3D &trans) { offset_R_L_I = rot; offset_T_L_I = trans; }
void ImuProcess::set_global_extrinsic_cov(const V3D &cr, const V3D &ct) { cov_global_extrinsic_rot = cr; cov_global_extrinsic_trans = ct; }

void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N) {
    printf("IMU Initializing: %.1f %%\n", double(N) / MAX_INI_COUNT * 100);
    V3D cur_acc, cur_gyr;
    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    }
    for (const auto &imu: meas.imu) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;
        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
        N++;
    }
    state_inout.gravity = - mean_acc / mean_acc.norm() * G_m_s2;
    state_inout.rot_end = Eye3d;
    state_inout.bias_g.setZero();
    last_imu_ = meas.imu.back();
}

// Propagation function unchanged — math logic is identical, only type changes from ROS1→ROS2
void ImuProcess::propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &orig_pcl_out) {
    orig_pcl_out = *(meas.lidar);
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    double imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
    double pcl_beg_time, pcl_end_time;
    if (lidar_type == SIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    } else {
        pcl_beg_time = meas.lidar_beg_time;
        pcl_end_time = pcl_beg_time + orig_pcl_out.points.back().curvature / double(1000);
    }
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));
    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);
    MatrixXd F_x, cov_w;
    F_x.resize(18, 18); cov_w.resize(DIM_STATE, DIM_STATE);
    double dt = 0.0;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);
        auto &&tail = *(it_imu + 1);
        if (rclcpp::Time(tail->header.stamp).seconds() < last_lidar_end_time_) continue;
        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr / IMU_mean_acc_norm * G_m_s2 - state_inout.bias_a;
        if (rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_)
            dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
        else
            dt = rclcpp::Time(tail->header.stamp).seconds() - rclcpp::Time(head->header.stamp).seconds();
        M3D acc_avr_skew;
        M3D Exp_f = Exp(angvel_avr, dt);
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);
        F_x.setIdentity();
        memset(cov_w.data(), 0, cov_w.size() * sizeof(double));
        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
        F_x.block<3, 3>(0, 9) = -Eye3d * dt;
        F_x.block<3, 3>(3, 6) = Eye3d * dt;
        F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
        F_x.block<3, 3>(6, 12) = -R_imu * dt;
        F_x.block<3, 3>(6, 15) = Eye3d * dt;
        cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
        cov_w.block<3, 3>(6, 6) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt;
        cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            cov_w.block<3, 3>(start_row, start_row).diagonal() = cov_global_extrinsic_rot * dt * dt;
            cov_w.block<3, 3>(start_row + 3, start_row + 3).diagonal() = cov_global_extrinsic_trans * dt * dt;
        }
        state_inout.cov.block<18,18>(0,0) = F_x * state_inout.cov.block<18,18>(0,0) * F_x.transpose();
        state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18) = F_x * state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18);
        state_inout.cov.block<6*MAX_UAV_NUM, 18>(18,0) = state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18).transpose();
        state_inout.cov += cov_w;
        R_imu = R_imu * Exp_f;
        acc_imu = R_imu * acc_avr + state_inout.gravity;
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
        vel_imu = vel_imu + acc_imu * dt;
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }
    unbiased_gyr = V3D(IMUpose.back().gyr[0], IMUpose.back().gyr[1], IMUpose.back().gyr[2]);
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    state_inout.vel_end = vel_imu + note * acc_imu * dt;
    state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
    state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;
    if (lidar_type != SIM) {
        auto it_pcl = orig_pcl_out.points.end() - 1;
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
            auto head = it_kp - 1;
            R_imu << MAT_FROM_ARRAY(head->rot);
            acc_imu << VEC_FROM_ARRAY(head->acc);
            vel_imu << VEC_FROM_ARRAY(head->vel);
            pos_imu << VEC_FROM_ARRAY(head->pos);
            angvel_avr << VEC_FROM_ARRAY(head->gyr);
            for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
                dt = it_pcl->curvature / double(1000) - head->offset_time;
                M3D R_i(R_imu * Exp(angvel_avr, dt));
                V3D P_i = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
                V3D p_in(it_pcl->x, it_pcl->y, it_pcl->z);
                V3D P_compensate = offset_R_L_I.transpose() * (state_inout.rot_end.transpose() * (R_i * (offset_R_L_I * p_in + offset_T_L_I) + P_i - state_inout.pos_end) - offset_T_L_I);
                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);
                if (it_pcl == orig_pcl_out.points.begin()) break;
            }
        }
    }
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr orig_pcl_un_) {
    if (meas.imu.empty()) return;
    if (meas.lidar == nullptr) return;
    if (imu_need_init_) {
        IMU_init(meas, stat, init_iter_num);
        imu_need_init_ = true;
        last_imu_ = meas.imu.back();
        if (init_iter_num > MAX_INI_COUNT) {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            printf("IMU Initialization Done: Gravity: %.4f %.4f %.4f, Acc norm: %.4f\n", stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm());
            IMU_mean_acc_norm = mean_acc.norm();
        }
        return;
    }
    propagation_and_undist(meas, stat, *orig_pcl_un_);
}
#endif
