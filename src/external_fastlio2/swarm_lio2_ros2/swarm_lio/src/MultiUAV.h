//
// Created by fangcheng on 2023/7/16.
// ROS2 Humble port by zengziheng 2026-07-11
//
#ifndef MULTIUAV_H
#define MULTIUAV_H

#include <omp.h>
#include <unistd.h>
#include <Eigen/Core>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <swarm_msgs/msg/quad_state_pub.hpp>
#include <swarm_msgs/msg/observe_teammate.hpp>
#include <swarm_msgs/msg/global_extrinsic_status.hpp>
#include <swarm_msgs/msg/global_extrinsic.hpp>
#include <swarm_msgs/msg/connected_teammate_list.hpp>
#include "common_lib.h"
#include <pcl/filters/filter.h>
#include "esikf_tracker.hpp"
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ExtrinsicInfection.hpp"
#include <algorithm>
#include <unordered_map>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/int8.hpp>
#include "FEC.h"
#include <tf2/LinearMath/Quaternion.h>

// ROS2 replacement for tf::createQuaternionMsgFromRollPitchYaw
inline geometry_msgs::msg::Quaternion createQuaternionFromRPY(double r, double p, double y) {
    tf2::Quaternion q; q.setRPY(r, p, y);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x(); msg.y = q.y(); msg.z = q.z(); msg.w = q.w();
    return msg;
}

typedef unordered_map<int, ESIKF> id_esikf_map;
typedef id_esikf_map::value_type id2ekf;

using namespace std;
using namespace Eigen;

class Multi_UAV {
public:
    template<class T> string SetString(T &param_in);
    Multi_UAV(rclcpp::Node* node, const int & drone_id_);
    ~Multi_UAV();

    struct QuadStateSub {
        QuadStateSub() {
            this->sub_time = 0.0; this->swarmlio_start_time = 0.0;
            this->rot_cov.setZero(); this->pos_cov.setZero();
            this->rot = M3D::Identity(); this->pos = Zero3d;
            this->gyr = Zero3d; this->vel = Zero3d;
            this->rot_end = M3D::Identity(); this->pos_end = Zero3d;
            this->my_pos_in_teammate = Zero3d;
            this->degenerated = false; this->is_observed = false;
        };
        QuadStateSub(const QuadStateSub &b) {
            this->sub_time = b.sub_time; this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov; this->pos_cov = b.pos_cov;
            this->rot = b.rot; this->pos = b.pos;
            this->gyr = b.gyr; this->vel = b.vel;
            this->rot_end = b.rot_end; this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated; this->is_observed = b.is_observed;
        };
        QuadStateSub& operator=(const QuadStateSub& b) {
            this->sub_time = b.sub_time; this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov; this->pos_cov = b.pos_cov;
            this->rot = b.rot; this->pos = b.pos;
            this->gyr = b.gyr; this->vel = b.vel;
            this->rot_end = b.rot_end; this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated; this->is_observed = b.is_observed;
            return *this;
        };
        double sub_time, swarmlio_start_time;
        M3D rot_cov, pos_cov, rot, rot_end;
        V3D pos, gyr, vel, pos_end, my_pos_in_teammate;
        bool degenerated, is_observed;
    };

    struct TemporaryTracker {
        TemporaryTracker(const ESIKF &tracker, const double &timestamp, const int &id) {
            this->dyn_tracker = tracker;
            Vector4d pos_and_time;
            pos_and_time.block<3,1>(0,0) = this->dyn_tracker.get_state_pos();
            pos_and_time(3) = timestamp;
            this->dyn_pos_time.push_back(pos_and_time);
            this->exist_meas = false; this->meas_of_tracker = Zero3d;
            this->create_time = timestamp; this->id = id;
        };
        TemporaryTracker(const TemporaryTracker &b) {
            this->dyn_tracker = b.dyn_tracker; this->dyn_pos_time = b.dyn_pos_time;
            this->exist_meas = b.exist_meas; this->meas_of_tracker = b.meas_of_tracker;
            this->create_time = b.create_time; this->id = b.id;
        };
        ESIKF dyn_tracker; deque<Vector4d> dyn_pos_time;
        bool exist_meas; V3D meas_of_tracker; double create_time; int id;
    };

    struct Cluster {
        Cluster(const V3D &pos, const bool &is_high_inten, const double &max_dist) {
            this->pos_in_body = pos; this->is_high_intensity = is_high_inten; this->max_dist = max_dist;
        };
        Cluster(const Cluster &b) {
            this->pos_in_body = b.pos_in_body; this->is_high_intensity = b.is_high_intensity; this->max_dist = b.max_dist;
        };
        V3D pos_in_body; bool is_high_intensity; double max_dist;
    };

    struct Teammate {
        Teammate() {
            teammate_pos_in_body = Zero3d; is_observe_teammate = false;
            last_connect_time = 0.0; first_connect_time = -1.0;
            teammate_odom_time.clear(); total_dist = 0.0; last_position = Zero3d;
            world_to_gravity_deg = Zero3d;
        };
        Teammate(const Teammate &b) {
            this->teammate_state = b.teammate_state; this->teammate_state_temp = b.teammate_state_temp;
            this->teammate_pos_in_body = b.teammate_pos_in_body;
            this->is_observe_teammate = b.is_observe_teammate;
            this->last_connect_time = b.last_connect_time;
            this->first_connect_time = b.first_connect_time;
            this->teammate_odom_time = b.teammate_odom_time;
            this->total_dist = b.total_dist; this->last_position = b.last_position;
            this->world_to_gravity_deg = b.world_to_gravity_deg;
        };
        QuadStateSub teammate_state, teammate_state_temp;
        V3D teammate_pos_in_body; bool is_observe_teammate;
        double last_connect_time, first_connect_time;
        deque<Vector4d> teammate_odom_time; double total_dist;
        V3D last_position, world_to_gravity_deg;
    };

    void BuildMatrixWithUpperTriangular(const VD(12) &vec, M3D &rot_cov, M3D &pos_cov);
    void QuadstateCbk(const swarm_msgs::msg::QuadStatePub::SharedPtr &msg);
    void GlobalExtrinsicCbk(const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr &msg);
    void ResetReconnectedGlobalExtrinsic(StatesGroup &state_in, const double &lidar_end_time);
    void UpdateFactorGraph(const bool &print_log);
    void UpdateGlobalExtrinsicAndCreateNewTeammateTracker(StatesGroup &state_in, const double &lidar_end_time);
    void CopyTeammateState(Teammate &teammate);
    void ResetTeammateState();
    void PublishQuadstate(const V3D &unbiased_gyr, const double &lidar_end_time, const double &first_lidar_time);
    void PublishGlobalExtrinsic(const double &lidar_end_time);
    template<class T> bool LoadParam(string param_name, T &param_value, T default_value);
    template<class T> bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);
    template<typename T> void SetPosestamp(T &out);
    bool IsObservedByTeammate(const Teammate &teammate);
    bool IsObserveTeammate(const Teammate &teammate);
    bool IsDurationShort(const double &lidar_end_time, Teammate &teammate, const int &id);
    void PropagateTeammateState(const double &lidar_end_time, Teammate &teammate);
    Matrix<double,(3),(12)> JacobianActiveObserve(V3D &active_observation_meas, const int &id, const Teammate &teammate);
    Matrix<double,(3),(12)> JacobComputeCase2(V3D &passive_observation_meas, const int &id, const Teammate &teammate);
    Matrix<double,(3),(12)> JacobianPassiveObserve(V3D &passive_observation_meas, const int &id, const Teammate &teammate, const double &lidar_end_time);
    M3D GetActiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double &mutual_observe_noise);
    M3D GetPassiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double & lidar_end_time, const double &mutual_observe_noise);
    void ClusterExtractPredictRegion(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);
    void ClusterExtractHighIntensity(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);
    void CheckClusterValidation(const int &id, Teammate &teammate);
    void PredictTracker(const double &lidar_end_time, const int &id);
    void UpdateTracker(const double &lidar_end_time, const int &id, const Teammate &teammate, const bool &cluster_meas, const bool &print_log);
    bool DeleteInvalidTemporaryTracker(const double &lidar_end_time, const int &index);
    bool TrajMatching(vector<Vector3d> &dyn_positions, vector<Vector3d> &uav_positions, Matrix3d &Rot, Vector3d &trans, double &Coeff, const int &id);
    bool CreateTeammateTracker(const double &lidar_end_time, const int &index, StatesGroup &state_in, StatesGroup &state_prop, const bool &print_log);
    void CreateTempTrackerByHighIntensity(const double &lidar_end_time);
    void CheckTempClusterValidation(const int &index);
    void PredictTemporaryTracker(const double &lidar_end_time, const int &index);
    void UpdateTemporaryTracker(const double &lidar_end_time, const int &index, const bool &print_log);
    void VisualizeText(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const double &time, const int &id, const double &scale, const V3D &pos, const string &text, const V3D &color);
    void VisualizeBoundingBox(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const double &time, const int &id, const V3D &color, const V3D &pos, const double &size);
    void VisualizeDeleteAllCluster(const double &time);
    void PublishTeammateOdom(const double &lidar_end_time);
    void PublishConnectedTeammateList(const double &lidar_end_time);
    void VisualizeTeammateTracker(const double &lidar_end_time, const int &teammate_id);
    void VisualizeDisconnectedTracker(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);
    void VisualizeMeshUAV(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);
    void VisualizeTempTracker(const double &lidar_end_time);
    void VisualizeTemporaryPredictRegion(const double &lidar_end_time);
    void VisualizeTempTrackerDelete(const double &lidar_end_time, const int &index);
    void VisualizeCluster(const double &lidar_end_time, const V3D &pos, const int &cluster_index, const V3D &size);
    void VisualizePredictRegion(const double &lidar_end_time, const int &id);
    void VisualizeRectangle(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_rect, const double &lidar_end_time, const int &rect_id, const V3D &position, const V3D rect_size);
    void VisualizeTeammateTrajectory(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_, deque<Vector4d> &traj, const V3D position, const string namespace_, const double mkr_size, const double &timestamp, const int teammate_id, const int has_observation);
    void VisualizeTeammateTrajectorySphere(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_, deque<Vector4d> &traj, const V3D position, const string namespace_, const double mkr_size, const double &timestamp, const int teammate_id, const int has_observation);

    id_esikf_map teammate_tracker;
    int pos_num_in_traj;
    vector<TemporaryTracker> temp_tracker;
    typedef unordered_map<int, Teammate> id_teammate_map;
    typedef id_teammate_map::value_type id2teammate;
    id_teammate_map teammates;
    string topic_name_prefix;
    M3D rot_world_to_gravity;
    StatesGroup state;
    ExtrinsicInfection extrinsic_infection;
    mutex mtx_buffer_reconnectID;
    bool degenerated;
    deque<Vector4d> teammate_traj[MAX_UAV_NUM];

private:
    rclcpp::Node* node_;
    rclcpp::Subscription<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_subscriber;
    rclcpp::Subscription<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_subscriber;
    rclcpp::Publisher<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_publisher;
    rclcpp::Publisher<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubUAV, pubMeshUAV, pubCluster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPredictRegionInput, pubHighIntenInput;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubPredictRegion, pubTempTracker;
    rclcpp::Publisher<swarm_msgs::msg::ConnectedTeammateList>::SharedPtr pubTeammateList;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubTeammateNum;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> pubTeammateOdom_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> pubTeammateTraj_;
    swarm_msgs::msg::QuadStatePub quadstate_msg_pub;
    swarm_msgs::msg::GlobalExtrinsicStatus global_extrinsic_msg;
    int drone_id, inten_threshold, min_high_inten_cluster_size, min_cluster_size, cluster_id{0};
    int lidar_type, actual_uav_num{4};
    vector<Cluster> cluster_pos_tag;
    vector<int> reconnected_id;
    vector<int> teammate_id_by_traj_matching;
    rclcpp::Publisher<swarm_msgs::msg::ConnectedTeammateList>::SharedPtr pubTeammateIdTrajMatching;
    double predict_region_radius, valid_cluster_dist_thresh, valid_cluster_size_thresh;
    double reset_tracker_thresh, temp_predict_region_radius;
    double valid_temp_cluster_dist_thresh, same_obj_thresh;
    double traj_matching_start_thresh, ave_match_error_thresh;
    nav_msgs::msg::Odometry TeammateOdom;
    bool found_all_teammates{false}, cluster_extraction_in_predict_region;
    double text_scale, mesh_scale;
};
#endif
