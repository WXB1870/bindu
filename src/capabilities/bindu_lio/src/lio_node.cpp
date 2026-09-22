// ROS 2 transport and input validation for the supplied FAST-LIO core.
// Algorithm attribution and source hashes: ../source_manifest.json.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <cstring>
#include <map>
#include <tuple>
#include "estimator.hpp"

namespace {
using Cloud = sensor_msgs::msg::PointCloud2;
using Imu = sensor_msgs::msg::Imu;
namespace core = bindu_fast_lio;

double seconds(const builtin_interfaces::msg::Time &stamp) {
    return rclcpp::Time(stamp).seconds();
}

Eigen::Isometry3d transform_parameter(rclcpp::Node &node, const std::string &name) {
    const auto values = node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (values.size() != 7 || !std::all_of(values.begin(), values.end(),
        [](double x) { return std::isfinite(x); })) {
        throw std::invalid_argument(name + " requires xyz and quaternion xyzw");
    }
    Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
    if (std::abs(q.norm() - 1.) > 1e-5) throw std::invalid_argument(name + " quaternion");
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.toRotationMatrix();
    result.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
    return result;
}

class LioNode final : public rclcpp::Node {
public:
    LioNode() : Node("fast_lio") {
        mapping_frame_ = declare_parameter<std::string>("mapping_frame", "");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
        imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
        if (odom_frame_.empty() || base_frame_.empty() || lidar_frame_.empty() ||
            imu_frame_.empty() || odom_frame_ == base_frame_) {
            throw std::invalid_argument("invalid LIO frames");
        }
        const auto imu_from_lidar = transform_parameter(*this, "imu_from_lidar");
        imu_from_base_ = transform_parameter(*this, "imu_from_base");
        max_age_ = positive("max_input_age", .5);
        max_imu_gap_ = positive("max_imu_gap", .05);
        max_acceleration_ = positive("max_imu_acceleration", 200.);
        max_angular_velocity_ = positive("max_imu_angular_velocity", 20.);
        max_scan_time_ = positive("max_scan_duration", .2);
        blind_ = positive("blind", .3);
        max_range_ = positive("max_range", 50.);
        map_voxel_ = positive("map_voxel", .2);
        const auto scan_voxel = positive("scan_voxel", .2);
        max_points_ = declare_parameter<int>("max_scan_points", 100000);
        max_map_points_ = declare_parameter<int>("max_map_points", 2000000);
        const int iterations = declare_parameter<int>("max_iterations", 4);
        map_path_ = declare_parameter<std::string>("save_map_path", "");
        if (max_points_ < 10 || max_points_ > 1000000 || max_map_points_ < 10 ||
            iterations < 1 || iterations > 20 || max_range_ <= blind_) {
            throw std::invalid_argument("invalid LIO limits");
        }
        core::configure(scan_voxel, map_voxel_, imu_from_lidar.translation(),
            imu_from_lidar.rotation(), iterations, positive("gyro_noise", .1),
            positive("accel_noise", .1), positive("gyro_bias_noise", .0001),
            positive("accel_bias_noise", .0001));
        tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("lio/odom", 4);
        cloud_pub_ = create_publisher<Cloud>("lio/cloud_registered", rclcpp::SensorDataQoS());
        status_pub_ = create_publisher<std_msgs::msg::String>("lio/status", 1);
        imu_sub_ = create_subscription<Imu>("lio/imu", rclcpp::SensorDataQoS().keep_last(256),
            [this](Imu::ConstSharedPtr msg) { receive_imu(*msg); });
        cloud_sub_ = create_subscription<Cloud>("lio/points", rclcpp::SensorDataQoS().keep_last(4),
            [this](Cloud::ConstSharedPtr msg) { receive_cloud(*msg); });
        timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { update(); });
        save_service_ = create_service<std_srvs::srv::Trigger>("lio/save_map",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) { save_map(*response); });
        report("WAITING_FOR_STATIONARY_IMU_AND_TIMED_CLOUD");
    }

private:
    double positive(const std::string &name, double fallback) {
        const auto value = declare_parameter<double>(name, fallback);
        if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument(name);
        return value;
    }

    void report(const std::string &code) {
        if (code == status_) return;
        status_ = code;
        std_msgs::msg::String message;
        message.data = code;
        status_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "%s", code.c_str());
    }

    void fault(const std::string &code) {
        faulted_ = true;
        clouds_.clear();
        imu_queue_.clear();
        report(code + ":RESTART_REQUIRED");
    }

    bool fresh(double stamp) const {
        const auto age = now().seconds() - stamp;
        return stamp > 0. && age >= -.01 && age <= max_age_;
    }

    void receive_imu(const Imu &message) {
        if (faulted_) return;
        const double stamp = seconds(message.header.stamp);
        if (message.header.frame_id != imu_frame_ || !fresh(stamp) || stamp <= last_imu_) {
            report("IMU_INPUT_REJECTED");
            return;
        }
        const auto &a = message.linear_acceleration;
        const auto &w = message.angular_velocity;
        const std::array<double, 6> values{a.x, a.y, a.z, w.x, w.y, w.z};
        if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); })) {
            report("IMU_NONFINITE");
            return;
        }
        if (Eigen::Vector3d(a.x, a.y, a.z).norm() > max_acceleration_ ||
            Eigen::Vector3d(w.x, w.y, w.z).norm() > max_angular_velocity_) {
            fault("IMU_RANGE_EXCEEDED");
            return;
        }
        if (last_imu_ > 0. && stamp - last_imu_ > max_imu_gap_) {
            fault("IMU_GAP");
            return;
        }
        if (!initialized_) {
            const double acceleration = Eigen::Vector3d(a.x, a.y, a.z).norm();
            if (Eigen::Vector3d(w.x, w.y, w.z).norm() > .15 || acceleration < 7. || acceleration > 12.) {
                fault("STATIONARY_SI_IMU_REQUIRED");
                return;
            }
        }
        auto sample = std::make_shared<LioImu>();
        sample->header.stamp.value = stamp;
        sample->linear_acceleration = {a.x, a.y, a.z};
        sample->angular_velocity = {w.x, w.y, w.z};
        imu_queue_.push_back(sample);
        last_imu_ = stamp;
        if (imu_queue_.size() > 1024) imu_queue_.pop_front();
    }

    // Canonical cloud contract: float32 x/y/z/intensity/time; time is the
    // per-point offset in seconds from the acquisition-start header stamp.
    // A Livox adapter must preserve offset_time; arrival time is never substituted.
    void receive_cloud(const Cloud &message) {
        if (faulted_) return;
        const double stamp = seconds(message.header.stamp);
        if (message.header.frame_id != lidar_frame_ || !fresh(stamp) || stamp <= last_cloud_) {
            report("CLOUD_INPUT_REJECTED");
            return;
        }
        const std::size_t count = std::size_t(message.width) * message.height;
        if (message.is_bigendian || !count || count > std::size_t(max_points_) ||
            message.point_step < 20 || message.row_step < std::uint64_t(message.width) * message.point_step ||
            message.data.size() != std::size_t(message.row_step) * message.height) {
            report("CLOUD_LAYOUT_REJECTED");
            return;
        }
        std::array<std::uint32_t, 5> offsets{};
        const std::array<std::string, 5> names{"x", "y", "z", "intensity", "time"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            auto field = std::find_if(message.fields.begin(), message.fields.end(),
                [&](const auto &f) { return f.name == names[i]; });
            if (field == message.fields.end() || field->datatype != 7 || field->count != 1 ||
                std::uint64_t(field->offset) + sizeof(float) > message.point_step) {
                report("TIMED_CLOUD_SCHEMA_REQUIRED");
                return;
            }
            offsets[i] = field->offset;
        }
        auto cloud = std::make_shared<PointCloudXYZI>();
        cloud->reserve(count);
        double duration = 0.;
        for (std::size_t i = 0; i < count; ++i) {
            const auto start = (i / message.width) * message.row_step + (i % message.width) * message.point_step;
            std::array<float, 5> values{};
            for (std::size_t j = 0; j < values.size(); ++j)
                std::memcpy(&values[j], &message.data[start + offsets[j]], sizeof(float));
            if (!std::all_of(values.begin(), values.end(), [](float x) { return std::isfinite(x); }) ||
                values[4] < 0. || values[4] > max_scan_time_) {
                report("CLOUD_VALUES_REJECTED");
                return;
            }
            duration = std::max(duration, double(values[4]));
            const double range = Eigen::Vector3d(values[0], values[1], values[2]).norm();
            if (range < blind_ || range > max_range_) continue;
            PointType point{};
            point.x = values[0]; point.y = values[1]; point.z = values[2];
            point.intensity = values[3]; point.curvature = values[4] * 1000.f;
            cloud->push_back(point);
        }
        if (cloud->size() < 10 || duration <= 0. || !fresh(stamp + duration)) {
            report("CLOUD_DURATION_OR_SIZE_REJECTED");
            return;
        }
        if (last_cloud_end_ > stamp + 1e-6) {
            fault("OVERLAPPING_SCANS");
            return;
        }
        if (last_cloud_end_ > 0. && stamp - last_cloud_end_ > max_age_) {
            fault("CLOUD_GAP");
            return;
        }
        std::sort(cloud->begin(), cloud->end(), [](const auto &a, const auto &b) { return a.curvature < b.curvature; });
        MeasureGroup measure;
        measure.lidar = cloud;
        measure.lidar_beg_time = stamp;
        measure.lidar_end_time = stamp + duration;
        clouds_.push_back(measure);
        last_cloud_ = stamp;
        last_cloud_end_ = measure.lidar_end_time;
        if (clouds_.size() > 4) fault("LIO_QUEUE_OVERFLOW");
    }

    void update() {
        if (faulted_) return;
        if (initialized_ && (!fresh(last_imu_) || !fresh(last_cloud_end_))) {
            fault("LIO_INPUT_STALE");
            return;
        }
        if (clouds_.empty() || imu_queue_.empty()) return;
        auto &measure = clouds_.front();
        if (!fresh(measure.lidar_end_time)) { fault("SCAN_PROCESSING_STALE"); return; }
        if (last_imu_ < measure.lidar_end_time) return;
        while (imu_queue_.size() > 1 && imu_queue_[1]->header.stamp.value <= measure.lidar_beg_time)
            imu_queue_.pop_front();
        if (imu_queue_.front()->header.stamp.value > measure.lidar_beg_time) {
            clouds_.pop_front();
            report("WAITING_FOR_IMU_COVERAGE");
            return;
        }
        for (const auto &imu : imu_queue_) {
            if (imu->header.stamp.value > measure.lidar_end_time) {
                const auto &previous = measure.imu.back();
                const double ratio = (measure.lidar_end_time - previous->header.stamp.value) /
                    (imu->header.stamp.value - previous->header.stamp.value);
                auto end = std::make_shared<LioImu>(*previous);
                end->header.stamp.value = measure.lidar_end_time;
                auto blend = [ratio](const LioVector &a, const LioVector &b) {
                    return LioVector{a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y),
                        a.z + ratio * (b.z - a.z)};
                };
                end->linear_acceleration = blend(previous->linear_acceleration, imu->linear_acceleration);
                end->angular_velocity = blend(previous->angular_velocity, imu->angular_velocity);
                measure.imu.push_back(end);
                break;
            }
            measure.imu.push_back(imu);
            if (imu->header.stamp.value == measure.lidar_end_time) break;
        }
        if (core::process(measure)) {
            if (!fresh(measure.lidar_end_time)) { fault("ESTIMATION_STALE"); return; }
            initialized_ = true;
            publish(measure);
        } else if (initialized_) {
            fault("LIO_MATCH_FAILED");
            return;
        }
        clouds_.pop_front();
    }

    void publish(const MeasureGroup &measure) {
        const auto &state = core::state_point;
        Eigen::Isometry3d odom_from_imu = Eigen::Isometry3d::Identity();
        odom_from_imu.linear() = state.rot.toRotationMatrix();
        odom_from_imu.translation() = state.pos;
        const Eigen::Isometry3d pose = odom_from_imu * imu_from_base_;
        const Eigen::Quaterniond q(pose.rotation());
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = rclcpp::Time(std::int64_t(measure.lidar_end_time * 1e9));
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = pose.translation().x();
        odom.pose.pose.position.y = pose.translation().y();
        odom.pose.pose.position.z = pose.translation().z();
        odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
        const auto &sample = measure.imu.back()->angular_velocity;
        const Eigen::Vector3d omega = Eigen::Vector3d(sample.x, sample.y, sample.z) - state.bg;
        const Eigen::Vector3d velocity = imu_from_base_.linear().transpose() *
            (odom_from_imu.linear().transpose() * state.vel + omega.cross(imu_from_base_.translation()));
        const Eigen::Vector3d angular = imu_from_base_.linear().transpose() * omega;
        odom.twist.twist.linear.x = velocity.x(); odom.twist.twist.linear.y = velocity.y();
        odom.twist.twist.linear.z = velocity.z();
        odom.twist.twist.angular.x = angular.x(); odom.twist.twist.angular.y = angular.y();
        odom.twist.twist.angular.z = angular.z();
        // Pose covariance is transformed from the estimator's position/right-SO(3)
        // error to base origin position and fixed odom-axis orientation errors.
        Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
        jacobian.block<3, 3>(0, 0).setIdentity();
        const Eigen::Vector3d lever = imu_from_base_.translation();
        Eigen::Matrix3d skew;
        skew << 0., -lever.z(), lever.y(), lever.z(), 0., -lever.x(), -lever.y(), lever.x(), 0.;
        jacobian.block<3, 3>(0, 3) = -odom_from_imu.linear() * skew;
        jacobian.block<3, 3>(3, 3) = odom_from_imu.linear();
        const Eigen::Matrix<double, 6, 6> covariance = jacobian * core::kf.get_P().block<6, 6>(0, 0) * jacobian.transpose();
        for (int row = 0; row < 6; ++row)
            for (int col = 0; col < 6; ++col) odom.pose.covariance[row * 6 + col] = covariance(row, col);
        odom_pub_->publish(odom);
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom.header; transform.child_frame_id = base_frame_;
        transform.transform.translation.x = pose.translation().x();
        transform.transform.translation.y = pose.translation().y();
        transform.transform.translation.z = pose.translation().z();
        transform.transform.rotation = odom.pose.pose.orientation;
        tf_->sendTransform(transform);
        if (!mapping_frame_.empty()) {
            // Map origin is defined by this estimator instance, not world truth.
            // Dynamic source-time TF expires if the mapper stops producing data.
            geometry_msgs::msg::TransformStamped origin;
            origin.header = odom.header;
            origin.header.frame_id = mapping_frame_;
            origin.child_frame_id = odom_frame_;
            origin.transform.rotation.w = 1.;
            tf_->sendTransform(origin);
        }
        Cloud registered;
        pcl::toROSMsg(*core::feats_down_world, registered);
        registered.header = odom.header;
        cloud_pub_->publish(registered);
        for (const auto &point : core::feats_down_world->points) {
            const auto key = std::make_tuple(int(std::floor(point.x / map_voxel_)),
                int(std::floor(point.y / map_voxel_)), int(std::floor(point.z / map_voxel_)));
            map_[key] = point;
        }
        if (map_.size() > std::size_t(max_map_points_)) fault("MAP_CAPACITY_REACHED");
        else report("TRACKING");
    }

    void save_map(std_srvs::srv::Trigger::Response &response) {
        if (map_path_.empty() || map_.empty()) { response.message = "MAP_PATH_AND_DATA_REQUIRED"; return; }
        const std::filesystem::path path(map_path_);
        const auto temporary = path.string() + ".partial";
        if (std::filesystem::exists(path) || std::filesystem::exists(temporary)) {
            response.message = "MAP_ALREADY_EXISTS"; return;
        }
        PointCloudXYZI cloud;
        for (const auto &entry : map_) cloud.push_back(entry.second);
        try {
            if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
            if (pcl::io::savePCDFileBinary(temporary, cloud) != 0) throw std::runtime_error("PCD_WRITE_FAILED");
            std::filesystem::rename(temporary, path);
            response.success = true;
            response.message = "SAVED_IN_LIO_ODOM_FRAME:" + path.string();
        } catch (const std::exception &error) { response.message = error.what(); }
    }

    std::string mapping_frame_, odom_frame_, base_frame_, lidar_frame_, imu_frame_, map_path_, status_;
    Eigen::Isometry3d imu_from_base_;
    double max_age_, max_imu_gap_, max_scan_time_, blind_, max_range_, map_voxel_;
    double last_imu_ = 0., last_cloud_ = 0., last_cloud_end_ = 0.;
    double max_acceleration_, max_angular_velocity_;
    int max_points_, max_map_points_;
    bool initialized_ = false, faulted_ = false;
    std::deque<LioImu::ConstPtr> imu_queue_;
    std::deque<MeasureGroup> clouds_;
    std::map<std::tuple<int, int, int>, PointType> map_;
    rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};
}  // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<LioNode>()); }
    catch (const std::exception &error) {
        RCLCPP_FATAL(rclcpp::get_logger("fast_lio"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
