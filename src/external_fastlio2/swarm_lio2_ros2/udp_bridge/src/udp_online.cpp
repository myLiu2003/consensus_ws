/**
 * @file udp_online.cpp
 * @brief ROS2 Humble port of Swarm-LIO2 UDP bridge
 *
 * Original ROS1 version: swarm_lio2/udp_bridge/
 * Changes:
 *   ROS 1 node handle  → inherit rclcpp::Node
 *   ROS 1 publishers   → rclcpp::Publisher<...>::SharedPtr
 *   ROS 1 subscribers  → rclcpp::Subscription<...>::SharedPtr
 *   ROS 1 timers       → rclcpp::TimerBase::SharedPtr
 *   ROS 1 timer events → removed; ROS2 timer callbacks take no arguments
 *   ROS 1 time         → rclcpp::Time
 *   ROS 1 duration     → std::chrono
 *   boost::thread      → std::thread
 *   boost::filesystem  → std::filesystem
 *   boost::bind        → std::bind / lambda
 *   ROS 1 async spinner → std::thread + rclcpp::spin
 *   ROS 1 log macros    → RCLCPP_WARN/ERROR
 *   ROS 1 package lookup → ament_index_cpp::get_package_share_directory
 *   MAVROS battery status → sensor_msgs::msg::BatteryState (already used)
 *
 * All UDP/socket/Eigen logic is unchanged.
 */

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "fmt/color.h"
#include "udp_bridge/protocol.h"
#include "algorithm"
#include <string>
#include <map>
#include <filesystem>
#include "swarm_msgs/msg/team_status.hpp"
#include "swarm_msgs/msg/teammate_info.hpp"
#include "swarm_msgs/msg/quad_state_pub.hpp"
#include "swarm_msgs/msg/observe_teammate.hpp"
#include "swarm_msgs/msg/global_extrinsic_status.hpp"
#include "swarm_msgs/msg/global_extrinsic.hpp"
#include "swarm_msgs/msg/spatial_temporal_offset.hpp"
#include "swarm_msgs/msg/spatial_temporal_offset_status.hpp"
#include "swarm_msgs/msg/connected_teammate_list.hpp"
#include <ifaddrs.h>
#include "Teammate.hpp"
#include <errno.h>
#include <ctime>
#include <csignal>
#include <unordered_map>
#include "so3_math.h"
#include <Eigen/Eigen>
#include "sensor_msgs/msg/battery_state.hpp"
#include "udp_bridge/scope_timer.hpp"
#include <numeric>
#include <std_msgs/msg/int8.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <net/if.h>

using namespace Eigen;
using namespace std;
using namespace fmt;
#define BUFFER_SIZE 10086
#define OBSERVE_MSG_TYPE 0x04u

using namespace udp_bridge;
typedef unordered_map<int, Teammate> id_teammate_map;
typedef id_teammate_map::value_type position;
bool exit_process = false;

void SigHandle(int sig) {
    printf("Exit the process, catch sig %d\n", sig);
    exit_process = true;
}

class UdpBridge : public rclcpp::Node {
private:
    int udp_server_fd_{-1};
    int udp_send_ip_fd_ptr_{-1};
    char udp_recv_buf_[BUFFER_SIZE];
    string local_ip, bind_ip_, log_dir_;
    int local_id;
    int udp_port_{8821};
    int drone_state = -1;
    id_teammate_map teammates;
    string broadcast_ip, offset_path;
    vector<string> peer_ips_;
    vector<sockaddr_in> discovery_peer_addrs_;
    ofstream save_offset;

    sockaddr_in addr_udp_send_ip_;
    thread udp_callback_thread_;  // std::thread 替代 boost::thread
    rclcpp::Publisher<swarm_msgs::msg::TeamStatus>::SharedPtr team_status_pub_;
    rclcpp::TimerBase::SharedPtr sync_timer_, broadcast_timer_, drone_state_timer_;
    rclcpp::Publisher<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_pub_;
    rclcpp::Publisher<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SpatialTemporalOffsetStatus>::SharedPtr ST_OffsetStatus_pub_;
    rclcpp::Subscription<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_sub_;
    rclcpp::Subscription<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr Battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr DroneState_sub_;
    rclcpp::Subscription<swarm_msgs::msg::ConnectedTeammateList>::SharedPtr TeammateListTraj_sub_;
    rclcpp::Time udp_start_time_;
    fstream log_writer_;
    ofstream fout_delay;
    vector<int> teammate_id_by_traj_matching;
    mutex traj_buffer_mtx;

    // ===== IpIdData =====
    struct IpIdData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        IpIdMsgCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT{0, 0, RCL_ROS_TIME};
    } ip_id_data_;

    void IpIdMsgCallback_() {
        if (!ip_id_data_.rcv_new_msg) { return; }
        ip_id_data_.update_lock_.lock();
        ip_id_data_.rcv_new_msg = false;
        ip_id_data_.processing_msg = ip_id_data_.latest_msg;
        ip_id_data_.update_lock_.unlock();

        uint8_t *data = ip_id_data_.processing_msg.data.local_ip;
        string rcv_ip;
        CharIp2StringIp(data, rcv_ip);

        int rcv_id = ip_id_data_.processing_msg.data.local_id;
        if (rcv_id == local_id) { return; }
        auto iter = teammates.find(rcv_id);
        double rcv_udp_start_time = ip_id_data_.processing_msg.data.udp_start_time.sec
            + ip_id_data_.processing_msg.data.udp_start_time.nsec * 1e-9;
        if (iter != teammates.end()) {
            if (abs(iter->second.udp_start_time_ - rcv_udp_start_time) > 1.0) {
                teammates.erase(iter);
                Teammate drone(rcv_ip, rcv_id, ip_id_data_.rcv_WT.seconds(), rcv_udp_start_time);
                drone.udp_send_fd_ptr_ = InitUdpUnicast(drone, udp_port_);
                teammates.insert(position(rcv_id, drone));
                print(fg(color::lime_green), " -- [Re-synchronize with Teammate]: UAV{}, {}\n", rcv_id, rcv_ip);
            } else {
                iter->second.last_rcv_time_ = ip_id_data_.rcv_WT.seconds();
            }
        } else {
            Teammate drone(rcv_ip, rcv_id, ip_id_data_.rcv_WT.seconds(), rcv_udp_start_time);
            drone.udp_send_fd_ptr_ = InitUdpUnicast(drone, udp_port_);
            teammates.insert(position(rcv_id, drone));
            print(fg(color::lime_green), " -- [Found New Teammate]: UAV{}, {}\n", rcv_id, rcv_ip);
        }
    }

    // ===== TimeSyncData =====
    struct TimeSyncData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        TimeSyncMsgCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT{0, 0, RCL_ROS_TIME};
    } time_sync_data_;

    void TimeSyncCallback_() {
        if (!time_sync_data_.rcv_new_msg) { return; }
        time_sync_data_.update_lock_.lock();
        time_sync_data_.rcv_new_msg = false;
        time_sync_data_.processing_msg = time_sync_data_.latest_msg;
        time_sync_data_.update_lock_.unlock();

        int rcv_client_id = time_sync_data_.processing_msg.data.client_id;
        int rcv_server_id = time_sync_data_.processing_msg.data.server_id;

        if (rcv_client_id == local_id) {
            auto iter = teammates.find(rcv_server_id);
            if (iter == teammates.end()) { return; }
            auto &drone = iter->second;
            if (drone.sync_done_) { return; }

            rclcpp::Time t4 = this->now();
            const auto clock_type = this->get_clock()->get_clock_type();
            rclcpp::Time t1(time_sync_data_.processing_msg.data.t1.sec * 1e9
                + time_sync_data_.processing_msg.data.t1.nsec, clock_type);
            rclcpp::Time t2(time_sync_data_.processing_msg.data.t2.sec * 1e9
                + time_sync_data_.processing_msg.data.t2.nsec, clock_type);
            rclcpp::Time t3(time_sync_data_.processing_msg.data.t3.sec * 1e9
                + time_sync_data_.processing_msg.data.t3.nsec, clock_type);
            double dt1 = (t2 - t1).seconds();
            double dt2 = (t4 - t3).seconds();
            double offset_t = 0.5 * dt1 - 0.5 * dt2;
            double dt = dt1 + dt2;

            drone.offset_ts_.push_back(offset_t);
            drone.delay_ts_.push_back(dt);
            if (drone.offset_ts_.size() >= 30) {
                drone.sync_done_ = true;
                drone.offset_time_ = accumulate(drone.offset_ts_.begin(), drone.offset_ts_.end(), 0.0)
                    / drone.offset_ts_.size();
                double delay_ = accumulate(drone.delay_ts_.begin(), drone.delay_ts_.end(), 0.0)
                    / drone.delay_ts_.size();
                print("Update offset with UAV{0}\n\tdelay = {1:.6} ms, offset time = {2:.6} ms\n",
                    drone.id_, delay_ * 1000, drone.offset_time_ * 1000);
            }
            return;
        }

        auto iter = teammates.find(rcv_client_id);
        if (iter != teammates.end()) {
            auto &drone = iter->second;
            time_sync_data_.processing_msg.data.t2.nsec = time_sync_data_.rcv_WT.nanoseconds() % 1000000000;
            time_sync_data_.processing_msg.data.t2.sec = time_sync_data_.rcv_WT.nanoseconds() / 1000000000;
            rclcpp::Time t3 = this->now();
            time_sync_data_.processing_msg.data.t3.sec = t3.nanoseconds() / 1000000000;
            time_sync_data_.processing_msg.data.t3.nsec = t3.nanoseconds() % 1000000000;
            int len = sizeof(time_sync_data_.processing_msg.binary) + 2 * sizeof(uint32_t);
            char send_buf[len * 5];
            EncodeMsgToBuffer(MESSAGE_TYPE::TIME_SYNC, time_sync_data_.processing_msg, send_buf);
            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
                (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(this->get_logger(), "UDP SEND BACK ERROR!!!");
            }
        }
    }

    // ===== QuadStateData =====
    struct QuadStateData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        QuadStateCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT{0, 0, RCL_ROS_TIME};
    } quad_state_data_;

    void QuadStateCallback_() {
        if (!quad_state_data_.rcv_new_msg) { return; }
        quad_state_data_.update_lock_.lock();
        quad_state_data_.processing_msg = quad_state_data_.latest_msg;
        quad_state_data_.update_lock_.unlock();

        swarm_msgs::msg::QuadStatePub quadstate_msg;
        quadstate_msg.teammate.clear();
        quadstate_msg.drone_id = quad_state_data_.processing_msg.data.drone_id;
        int rcv_id = quadstate_msg.drone_id;
        static int cnt[MAX_UAV_NUM] = {0};
        cnt[rcv_id]++;

        auto iter = teammates.find(rcv_id);
        if (iter != teammates.end() && iter->second.sync_done_) {
            double dt_ns = iter->second.offset_time_ * 1e9;
            if (cnt[rcv_id] % 30 == 0)
                cout << "Fuse msg from UAV" << int(rcv_id) << ", offset time: " << dt_ns / 1e6 << " ms" << endl;
            uint64_t raw_ns = quad_state_data_.processing_msg.data.header.sec * 1000000000ULL
                + quad_state_data_.processing_msg.data.header.nsec - (int64_t)dt_ns;
            quadstate_msg.header.stamp.sec = raw_ns / 1000000000;
            quadstate_msg.header.stamp.nanosec = raw_ns % 1000000000;
            quadstate_msg.header.frame_id = "world";
            quadstate_msg.pose.header.frame_id = "world";
            quadstate_msg.pose.pose.orientation.w = quad_state_data_.processing_msg.data.quat_w;
            quadstate_msg.pose.pose.orientation.x = quad_state_data_.processing_msg.data.quat_x;
            quadstate_msg.pose.pose.orientation.y = quad_state_data_.processing_msg.data.quat_y;
            quadstate_msg.pose.pose.orientation.z = quad_state_data_.processing_msg.data.quat_z;
            quadstate_msg.pose.pose.position.x = quad_state_data_.processing_msg.data.pos[0];
            quadstate_msg.pose.pose.position.y = quad_state_data_.processing_msg.data.pos[1];
            quadstate_msg.pose.pose.position.z = quad_state_data_.processing_msg.data.pos[2];
            for (int i = 0; i < 3; i++) {
                quadstate_msg.vel[i] = quad_state_data_.processing_msg.data.vel[i];
                quadstate_msg.gyr[i] = quad_state_data_.processing_msg.data.gyr[i];
                quadstate_msg.world_to_gravity_deg[i] = quad_state_data_.processing_msg.data.world_to_gravity_deg[i];
            }
            for (int i = 0; i < 12; ++i)
                quadstate_msg.pose_cov[i] = quad_state_data_.processing_msg.data.pose_cov[i];
            quadstate_msg.swarmlio_start_time = quad_state_data_.processing_msg.data.swarmlio_start_time;
            quadstate_msg.degenerated = quad_state_data_.processing_msg.data.degenerated;
            for (int i = 0; i < MAX_UAV_NUM; i++) {
                if (quad_state_data_.processing_msg.data.teammate[i].is_observe) {
                    swarm_msgs::msg::ObserveTeammate obs_teammate;
                    obs_teammate.is_observe = true;
                    obs_teammate.teammate_id = quad_state_data_.processing_msg.data.teammate[i].teammate_id;
                    for (int j = 0; j < 3; j++)
                        obs_teammate.observed_pos[j] = quad_state_data_.processing_msg.data.teammate[i].observed_pos[j];
                    quadstate_msg.teammate.push_back(obs_teammate);
                }
            }
            QuadState_pub_->publish(quadstate_msg);
        }
        quad_state_data_.rcv_new_msg = false;
    }

    // ===== GlobalExtrinsicStatusData =====
    struct GlobalExtrinsicStatusData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        GlobalExtrinsicStatusCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT{0, 0, RCL_ROS_TIME};
    } global_extrinsic_data_;

    void GlobalExtrinsicCallback_() {
        if (!global_extrinsic_data_.rcv_new_msg) { return; }
        global_extrinsic_data_.update_lock_.lock();
        global_extrinsic_data_.processing_msg = global_extrinsic_data_.latest_msg;
        global_extrinsic_data_.update_lock_.unlock();

        swarm_msgs::msg::GlobalExtrinsicStatus global_extrinsic_status_msg;
        global_extrinsic_status_msg.extrinsic.clear();
        global_extrinsic_status_msg.drone_id = global_extrinsic_data_.processing_msg.data.drone_id;
        int rcv_id = global_extrinsic_status_msg.drone_id;
        auto iter = teammates.find(rcv_id);
        if (iter != teammates.end() && iter->second.sync_done_) {
            double dt_ns = iter->second.offset_time_ * 1e9;
            uint64_t raw_ns = global_extrinsic_data_.processing_msg.data.header.sec * 1000000000ULL
                + global_extrinsic_data_.processing_msg.data.header.nsec - (int64_t)dt_ns;
            global_extrinsic_status_msg.header.stamp.sec = raw_ns / 1000000000;
            global_extrinsic_status_msg.header.stamp.nanosec = raw_ns % 1000000000;
            global_extrinsic_status_msg.header.frame_id = "world";
            for (int i = 0; i < MAX_UAV_NUM; i++) {
                swarm_msgs::msg::GlobalExtrinsic global_extrinsic;
                global_extrinsic.teammate_id = global_extrinsic_data_.processing_msg.data.extrinsic[i].teammate_id;
                if (int(global_extrinsic.teammate_id) > MAX_UAV_NUM) continue;
                for (int j = 0; j < 3; ++j) {
                    global_extrinsic.rot_deg[j] = global_extrinsic_data_.processing_msg.data.extrinsic[i].rot_deg[j];
                    global_extrinsic.trans[j] = global_extrinsic_data_.processing_msg.data.extrinsic[i].trans[j];
                }
                global_extrinsic_status_msg.extrinsic.push_back(global_extrinsic);
            }
            GlobalExtrinsic_pub_->publish(global_extrinsic_status_msg);
        }
        global_extrinsic_data_.rcv_new_msg = false;
    }

    void initDataCallback() {
        ip_id_data_.process_timer = this->create_wall_timer(
            chrono::milliseconds(1), [this]() { IpIdMsgCallback_(); });
        time_sync_data_.process_timer = this->create_wall_timer(
            chrono::milliseconds(1), [this]() { TimeSyncCallback_(); });
        quad_state_data_.process_timer = this->create_wall_timer(
            chrono::milliseconds(1), [this]() { QuadStateCallback_(); });
        global_extrinsic_data_.process_timer = this->create_wall_timer(
            chrono::milliseconds(1), [this]() { GlobalExtrinsicCallback_(); });
    }

public:
    UdpBridge() : Node("udp_soft_time_sync") {
        local_id = this->declare_parameter<int>("drone_id", 1);
        bind_ip_ = this->declare_parameter<string>("bind_ip", "");
        broadcast_ip = this->declare_parameter<string>("broadcast_ip", "255.255.255.255");
        peer_ips_ = this->declare_parameter<vector<string>>("peer_ips", {});
        udp_port_ = this->declare_parameter<int>("port", 8821);
        log_dir_ = this->declare_parameter<string>("log_dir", "/tmp/swarm_lio/udp");

        if (local_id < 0 || local_id > 31) {
            throw std::invalid_argument("drone_id must be in range 0..31");
        }
        if (udp_port_ < 1 || udp_port_ > 65535) {
            throw std::invalid_argument("port must be in range 1..65535");
        }
        local_ip = DiscoverLocalIpv4(bind_ip_);
        if (local_ip.empty()) {
            throw std::runtime_error("No usable IPv4 address; set the bind_ip parameter explicitly");
        }

        print(fg(color::lime_green), " -- [BROAD IP]: {}\n", broadcast_ip);
        print(fg(color::lime_green), " -- [LOCAL IP]: {}\n", local_ip);
        print(fg(color::lime_green), " -- [DRONE ID]: {}\n", local_id);

        offset_path = log_dir_;
        filesystem::create_directories(log_dir_);
        offset_path += "/teammate_" + GetSystemTime() + ".txt";
        save_offset.open(offset_path, ios::out);

        teammate_id_by_traj_matching.clear();

        udp_send_ip_fd_ptr_ = InitUdpBoardcast(udp_port_);
        for (const auto &peer_ip : peer_ips_) {
            if (peer_ip == local_ip) continue;
            sockaddr_in peer_addr{};
            peer_addr.sin_family = AF_INET;
            peer_addr.sin_port = htons(udp_port_);
            if (inet_pton(AF_INET, peer_ip.c_str(), &peer_addr.sin_addr) <= 0) {
                throw std::invalid_argument("peer_ips contains invalid IPv4 address: " + peer_ip);
            }
            discovery_peer_addrs_.push_back(peer_addr);
        }
        udp_callback_thread_ = thread(&UdpBridge::UdpCallback, this);
        broadcast_timer_ = this->create_wall_timer(chrono::seconds(1), [this]() { BroadcastCallback(); });
        sync_timer_ = this->create_wall_timer(chrono::milliseconds(100), [this]() { SyncRequestCallback(); });
        drone_state_timer_ = this->create_wall_timer(chrono::milliseconds(200), [this]() { DroneStateTimerCallback(); });

        team_status_pub_ = this->create_publisher<swarm_msgs::msg::TeamStatus>("/team_status", 1000);
        QuadState_pub_ = this->create_publisher<swarm_msgs::msg::QuadStatePub>("/quadstate_from_teammate", 1000);
        QuadState_sub_ = this->create_subscription<swarm_msgs::msg::QuadStatePub>(
            "/quadstate_to_teammate", 1000, [this](const swarm_msgs::msg::QuadStatePub::SharedPtr m) { QuadStateCallback(m); });
        GlobalExtrinsic_pub_ = this->create_publisher<swarm_msgs::msg::GlobalExtrinsicStatus>("/global_extrinsic_from_teammate", 1000);
        GlobalExtrinsic_sub_ = this->create_subscription<swarm_msgs::msg::GlobalExtrinsicStatus>(
            "/global_extrinsic_to_teammate", 1000, [this](const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr m) { GlobalExtrinsicCallback(m); });
        TeammateListTraj_sub_ = this->create_subscription<swarm_msgs::msg::ConnectedTeammateList>(
            "/teammate_id_with_traj_matching", 1000, [this](const swarm_msgs::msg::ConnectedTeammateList::SharedPtr m) { TeammateListTrajCallback(m); });
        ST_OffsetStatus_pub_ = this->create_publisher<swarm_msgs::msg::SpatialTemporalOffsetStatus>("/spatial_temporal_offset", 1000);
        Battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 1000, [this](const sensor_msgs::msg::BatteryState::SharedPtr m) { BatteryStatusCallback(m); });
        DroneState_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/mpc/drone_state", 1000, [this](const std_msgs::msg::Int8::SharedPtr m) { DroneStateCallback(m); });

        log_writer_.open(log_dir_ + "/udp_debug.txt", ios::out);
        fout_delay.open(log_dir_ + "/udp_delay.txt", ios::out);
        initDataCallback();
        udp_start_time_ = this->now();
        rclcpp::sleep_for(chrono::milliseconds(100));
    }

    ~UdpBridge() {
        if (udp_server_fd_ >= 0) {
            shutdown(udp_server_fd_, SHUT_RDWR);
            close(udp_server_fd_);
            udp_server_fd_ = -1;
        }
        if (udp_callback_thread_.joinable()) udp_callback_thread_.join();
        if (udp_send_ip_fd_ptr_ >= 0) close(udp_send_ip_fd_ptr_);
        log_writer_.close();
        fout_delay.close();
    }

    void BroadcastCallback() {
        if (exit_process) {
            for (auto it = teammates.begin(); it != teammates.end(); it++) {
                auto &drone = it->second;
                if (!drone.write_done_) {
                    if (drone.sync_done_) {
                        save_offset << local_id << " " << drone.id_ << " " << drone.offset_time_ << " " << drone.ip_ << endl;
                        drone.write_done_ = true;
                    }
                }
            }
            save_offset.close();
            rclcpp::shutdown();
        }
        SendLocalIp();

        swarm_msgs::msg::TeamStatus team_msg;
        team_msg.my_drone_id = local_id;
        for (auto it = teammates.begin(); it != teammates.end(); it++) {
            swarm_msgs::msg::TeammateInfo teammate_info_msg;
            teammate_info_msg.is_connect = false;
            auto &drone = it->second;
            double cur_time = this->now().seconds();
            if (drone.is_connect(cur_time))
                teammate_info_msg.is_connect = true;
            teammate_info_msg.id = drone.id_;
            uint8_t *ip_c2 = new uint8_t[4];
            StringIp2CharIp(drone.ip_, ip_c2);
            for (int i = 0; i < 4; i++)
                teammate_info_msg.ip[i] = ip_c2[i];
            team_msg.teammate_info.push_back(teammate_info_msg);
        }
        team_status_pub_->publish(team_msg);
    }

    void SyncRequestCallback() {
        for (auto it = teammates.begin(); it != teammates.end(); it++) {
            auto &drone = it->second;
            if (!drone.sync_done_) {
                if (!drone.offset_ts_.empty())
                    print(" -- [Sync to] UAV{} {}/30.\n", drone.id_, drone.offset_ts_.size());
                CallSyncRequest(drone);
                rclcpp::sleep_for(chrono::milliseconds(10));
            }
        }
    }

    void UdpCallback() {
        int valread;
        struct sockaddr_in addr_client;
        socklen_t addr_len = sizeof(addr_client);

        if (BindToUdpPort(udp_port_, udp_server_fd_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "[bridge_node]Socket receiver creation error!");
            exit(EXIT_FAILURE);
        }

        while (rclcpp::ok()) {
            if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUFFER_SIZE, 0,
                (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0) {
                if (!rclcpp::ok() || exit_process) break;
                perror("recvfrom() < 0, error:");
                continue;
            }
            rclcpp::Time t2 = this->now();
            char *ptr = udp_recv_buf_;
            switch (*((MESSAGE_TYPE *)ptr)) {
                case MESSAGE_TYPE::IP_ID: {
                    IpIdMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    ip_id_data_.update_lock_.lock();
                    ip_id_data_.latest_msg = rcv_msg;
                    ip_id_data_.rcv_WT = t2;
                    ip_id_data_.rcv_new_msg = true;
                    ip_id_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::TIME_SYNC: {
                    TimeSyncMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    time_sync_data_.update_lock_.lock();
                    time_sync_data_.latest_msg = rcv_msg;
                    time_sync_data_.rcv_WT = t2;
                    time_sync_data_.rcv_new_msg = true;
                    time_sync_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::QUAD_STATE: {
                    QuadStateCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    quad_state_data_.update_lock_.lock();
                    quad_state_data_.latest_msg = rcv_msg;
                    quad_state_data_.rcv_WT = t2;
                    quad_state_data_.rcv_new_msg = true;
                    quad_state_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::GLOBAL_EXTRINSIC: {
                    GlobalExtrinsicStatusCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    global_extrinsic_data_.update_lock_.lock();
                    global_extrinsic_data_.latest_msg = rcv_msg;
                    global_extrinsic_data_.rcv_WT = t2;
                    global_extrinsic_data_.rcv_new_msg = true;
                    global_extrinsic_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::GCS_CMD: {
                    cout << "Received GCS Msg" << endl;
                    GCSCmdMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    // GCS CMD not implemented in ROS2 (system() calls are unsafe)
                    break;
                }
                default:
                    break;
            }
        }
    }

    // ===== Send/Receive/Network helpers (unchanged from ROS1) =====
    void CallSyncRequest(const Teammate &drone) {
        TimeSyncMsgCvt cvt;
        cvt.data.client_id = local_id;
        cvt.data.server_id = drone.id_;
        rclcpp::Time t1 = this->now();
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        cvt.data.t1.sec = t1.nanoseconds() / 1000000000;
        cvt.data.t1.nsec = t1.nanoseconds() % 1000000000;
        EncodeMsgToBuffer(MESSAGE_TYPE::TIME_SYNC, cvt, send_buf);
        if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
            (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP SEND ERROR!!!");
            printf("errno is: %d\n", errno);
        }
    }

    void SendLocalIp() {
        IpIdMsgCvt cvt;
        StringIp2CharIp(local_ip, cvt.data.local_ip);
        cvt.data.local_id = local_id;
        cvt.data.udp_start_time.sec = udp_start_time_.nanoseconds() / 1000000000;
        cvt.data.udp_start_time.nsec = udp_start_time_.nanoseconds() % 1000000000;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::IP_ID, cvt, send_buf);
        if (sendto(udp_send_ip_fd_ptr_, send_buf, len, 0,
            (struct sockaddr *)&addr_udp_send_ip_, sizeof(addr_udp_send_ip_)) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP BROADCAST ERROR !!!");
        }
        for (const auto &peer_addr : discovery_peer_addrs_) {
            if (sendto(udp_send_ip_fd_ptr_, send_buf, len, 0,
                (const struct sockaddr *)&peer_addr, sizeof(peer_addr)) <= 0) {
                RCLCPP_ERROR(this->get_logger(), "UDP PEER DISCOVERY ERROR !!!");
            }
        }
    }

    template<typename union_msg>
    int EncodeMsgToBuffer(const MESSAGE_TYPE msg_type_id, union_msg &msg, char *send_buf_) {
        uint32_t msg_size = sizeof(msg.binary);
        int len = msg_size + 2 * sizeof(uint32_t);
        auto ptr = (uint8_t *)(send_buf_);
        *((MESSAGE_TYPE *)ptr) = msg_type_id;
        ptr += sizeof(MESSAGE_TYPE);
        *((uint32_t *)ptr) = msg_size;
        ptr += sizeof(uint32_t);
        memcpy(ptr, msg.binary, msg_size);
        return len;
    }

    template<typename union_msg>
    int DecodeMsgFromBuffer(union_msg &msg) {
        auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(uint32_t));
        uint32_t msg_size = *((uint32_t *)ptr);
        ptr += sizeof(uint32_t);
        memcpy(msg.binary, ptr, msg_size);
        return msg_size + sizeof(uint32_t) * 2;
    }

    int InitUdpUnicast(Teammate &drone, const int &port) {
        string ip_s = drone.ip_;
        const char *ip = ip_s.c_str();
        int fd;
        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "[udp_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }
        drone.addr_udp_send_.sin_family = AF_INET;
        drone.addr_udp_send_.sin_port = htons(port);
        if (inet_pton(AF_INET, ip, &drone.addr_udp_send_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
        return fd;
    }

    int InitUdpBoardcast(const int port) {
        int fd;
        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "[udp_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }
        int so_broadcast = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0) {
            cout << "Error in setting Broadcast option";
            exit(EXIT_FAILURE);
        }
        addr_udp_send_ip_.sin_family = AF_INET;
        addr_udp_send_ip_.sin_port = htons(port);
        if (inet_pton(AF_INET, broadcast_ip.c_str(), &addr_udp_send_ip_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
        return fd;
    }

    int BindToUdpPort(const int port, int &server_fd) {
        struct sockaddr_in address;
        int opt = 1;
        if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
            perror("setsockopt SO_REUSEADDR");
            exit(EXIT_FAILURE);
        }
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt))) {
            perror("setsockopt SO_REUSEPORT");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        if (bind_ip_.empty() || bind_ip_ == "0.0.0.0") {
            address.sin_addr.s_addr = INADDR_ANY;
        } else if (inet_pton(AF_INET, bind_ip_.c_str(), &address.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid bind_ip: %s", bind_ip_.c_str());
            return -1;
        }
        address.sin_port = htons(port);
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        return server_fd;
    }

    string DiscoverLocalIpv4(const string &requested_ip) {
        if (!requested_ip.empty() && requested_ip != "0.0.0.0") {
            in_addr address{};
            if (inet_pton(AF_INET, requested_ip.c_str(), &address) == 1) return requested_ip;
            throw std::invalid_argument("bind_ip is not a valid IPv4 address: " + requested_ip);
        }

        string fallback;
        ifaddrs *interfaces = nullptr;
        if (getifaddrs(&interfaces) != 0) return fallback;
        for (ifaddrs *entry = interfaces; entry != nullptr; entry = entry->ifa_next) {
            if (entry->ifa_addr == nullptr || entry->ifa_addr->sa_family != AF_INET) continue;
            if ((entry->ifa_flags & IFF_LOOPBACK) != 0) continue;
            char address[INET_ADDRSTRLEN]{};
            auto *ipv4 = reinterpret_cast<sockaddr_in *>(entry->ifa_addr);
            if (inet_ntop(AF_INET, &ipv4->sin_addr, address, sizeof(address)) == nullptr) continue;
            string candidate(address);
            if (fallback.empty()) fallback = candidate;
            if (candidate.rfind("192.168.", 0) == 0 || candidate.rfind("10.", 0) == 0) {
                freeifaddrs(interfaces);
                return candidate;
            }
        }
        freeifaddrs(interfaces);
        return fallback;
    }

    void TeammateListTrajCallback(const swarm_msgs::msg::ConnectedTeammateList::SharedPtr msg) {
        if (teammate_id_by_traj_matching.size() != msg->connected_teammate_id.size()) {
            traj_buffer_mtx.lock();
            teammate_id_by_traj_matching.clear();
            for (size_t i = 0; i < msg->connected_teammate_id.size(); i++)
                teammate_id_by_traj_matching.push_back(msg->connected_teammate_id[i]);
            traj_buffer_mtx.unlock();
        }
    }

    void GlobalExtrinsicCallback(const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr msg) {
        GlobalExtrinsicStatusCvt cvt;
        cvt.data.header.sec = msg->header.stamp.sec;
        cvt.data.header.nsec = msg->header.stamp.nanosec;
        cvt.data.drone_id = msg->drone_id;
        for (size_t i = 0; i < teammate_id_by_traj_matching.size(); i++)
            cvt.data.extrinsic[i].teammate_id = 255;
        int id_counter = 0;
        for (size_t i = 0; i < msg->extrinsic.size(); i++) {
            traj_buffer_mtx.lock();
            auto iter = find(teammate_id_by_traj_matching.begin(), teammate_id_by_traj_matching.end(),
                msg->extrinsic[i].teammate_id);
            if (iter == teammate_id_by_traj_matching.end()) { traj_buffer_mtx.unlock(); continue; }
            traj_buffer_mtx.unlock();
            cvt.data.extrinsic[id_counter].teammate_id = msg->extrinsic[i].teammate_id;
            for (int j = 0; j < 3; ++j) {
                cvt.data.extrinsic[id_counter].rot_deg[j] = msg->extrinsic[i].rot_deg[j];
                cvt.data.extrinsic[id_counter].trans[j] = msg->extrinsic[i].trans[j];
            }
            id_counter++;
        }
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::GLOBAL_EXTRINSIC, cvt, send_buf);
        static int cnt_ext[MAX_UAV_NUM] = {0};
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            cnt_ext[drone.id_]++;
            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
                (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Global Extrinsic SEND ERROR !!!");
            } else if (cnt_ext[drone.id_] % 10 == 0)
                cout << "UAV" << int(msg->drone_id) << " Send Global Extrinsic to UAV" << drone.id_ << endl;
        }

        swarm_msgs::msg::SpatialTemporalOffsetStatus st_offset_status;
        st_offset_status.st_offset.clear();
        st_offset_status.header.stamp = msg->header.stamp;
        st_offset_status.drone_id = msg->drone_id;
        for (size_t i = 0; i < msg->extrinsic.size(); i++) {
            swarm_msgs::msg::SpatialTemporalOffset st_offset;
            st_offset.teammate_id = msg->extrinsic[i].teammate_id;
            auto iter2 = teammates.find(int(st_offset.teammate_id));
            if (iter2 != teammates.end()) {
                st_offset.time_offset = iter2->second.offset_time_;
                st_offset_status.st_offset.push_back(st_offset);
            }
        }
        ST_OffsetStatus_pub_->publish(st_offset_status);
    }

    void QuadStateCallback(const swarm_msgs::msg::QuadStatePub::SharedPtr msg) {
        QuadStateCvt cvt;
        cvt.data.header.sec = msg->header.stamp.sec;
        cvt.data.header.nsec = msg->header.stamp.nanosec;
        cvt.data.drone_id = msg->drone_id;
        cvt.data.quat_w = msg->pose.pose.orientation.w;
        cvt.data.quat_x = msg->pose.pose.orientation.x;
        cvt.data.quat_y = msg->pose.pose.orientation.y;
        cvt.data.quat_z = msg->pose.pose.orientation.z;
        cvt.data.pos[0] = msg->pose.pose.position.x;
        cvt.data.pos[1] = msg->pose.pose.position.y;
        cvt.data.pos[2] = msg->pose.pose.position.z;
        for (int i = 0; i < 3; i++) { cvt.data.gyr[i] = msg->gyr[i]; cvt.data.vel[i] = msg->vel[i];
            cvt.data.world_to_gravity_deg[i] = msg->world_to_gravity_deg[i]; }
        for (int i = 0; i < 12; ++i) cvt.data.pose_cov[i] = msg->pose_cov[i];
        cvt.data.degenerated = msg->degenerated;
        cvt.data.swarmlio_start_time = msg->swarmlio_start_time;
        for (int i = 0; i < MAX_UAV_NUM; i++) cvt.data.teammate[i].is_observe = false;
        for (size_t i = 0; i < msg->teammate.size(); i++) {
            cvt.data.teammate[i].is_observe = msg->teammate[i].is_observe;
            cvt.data.teammate[i].teammate_id = msg->teammate[i].teammate_id;
            for (int j = 0; j < 3; j++) cvt.data.teammate[i].observed_pos[j] = msg->teammate[i].observed_pos[j];
        }
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::QUAD_STATE, cvt, send_buf);
        static int cnt_qs[MAX_UAV_NUM] = {0};
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            cnt_qs[drone.id_]++;
            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
                (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(this->get_logger(), "QUADSTATE SEND ERROR !!!");
            } else if (cnt_qs[drone.id_] % 50 == 0)
                cout << "UAV" << int(msg->drone_id) << ", sent quadstate to UAV" << drone.id_ << endl;
        }
    }

    void BatteryStatusCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        if (msg->voltage <= 20.4 || msg->voltage >= 25.2) return;
        float remaining = (msg->voltage - 20.4) / (25.2 - 20.4) * 100;
        BatteryStatusMsgCvt cvt;
        cvt.data.remaining = int(remaining);
        cvt.data.drone_id = local_id;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::BATTERY_STATUS, cvt, send_buf);
        static int cnt_bat = 0;
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            if (drone.id_ == 0) {
                cnt_bat++;
                if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
                    (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0)
                    RCLCPP_ERROR(this->get_logger(), "QUADSTATE SEND ERROR !!!");
                else if (cnt_bat % 20 == 0)
                    cout << "UAV" << int(local_id) << ", sent battery info to Ground Station." << endl;
                break;
            }
        }
    }

    void DroneStateTimerCallback() {
        Drone_StateMsgCvt cvt;
        cvt.data.drone_id = local_id;
        cvt.data.drone_state = drone_state;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::DRONE_STATE, cvt, send_buf);
        static int cnt_ds = 0;
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            if (drone.id_ == 0) {
                if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0,
                    (struct sockaddr *)&drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0)
                    RCLCPP_ERROR(this->get_logger(), "QUADSTATE SEND ERROR !!!");
                else if (cnt_ds % 5 == 0) { cnt_ds++;
                    cout << "UAV" << int(local_id) << ", sent drone state to Ground Station." << endl; }
                break;
            }
        }
    }

    void DroneStateCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        drone_state = msg->data;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    auto node = make_shared<UdpBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
