//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#if __has_include(<livox_ros_driver/CustomMsg.h>)
#include <livox_ros_driver/CustomMsg.h>
#define LIVOX_CUSTOM_MSG_AVAILABLE 1
#else
#define LIVOX_CUSTOM_MSG_AVAILABLE 0
#endif
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"

namespace gazebo {

static constexpr int kSensorPointCloud = 1;
static constexpr int kSensorPointCloud2 = 2;
static constexpr int kLivoxCustomMsg = 3;

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (size_t i = 0; i < datas.size(); ++i) {
        const auto &data = datas[i];
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
            avia_infos.back().line = static_cast<int>(i % 6);
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    sdfPtr = sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");

    int argc = 0;
    char **argv = nullptr;
    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
    ROS_INFO_STREAM("ros topic name:" << curr_scan_topic);
    ros::init(argc, argv, curr_scan_topic);
    rosNode.reset(new ros::NodeHandle);
    if (sdfPtr->HasElement("publish_pointcloud_type")) {
        publishPointCloudType = sdfPtr->Get<int>("publish_pointcloud_type");
    }
    if (publishPointCloudType == kLivoxCustomMsg && !LIVOX_CUSTOM_MSG_AVAILABLE) {
        ROS_WARN_STREAM("livox_ros_driver::CustomMsg 未找到，自动回退到 PointCloud2 输出。");
        publishPointCloudType = kSensorPointCloud2;
    }
    if (publishPointCloudType != kSensorPointCloud && publishPointCloudType != kSensorPointCloud2 &&
        publishPointCloudType != kLivoxCustomMsg) {
        ROS_WARN_STREAM("Unsupported publish_pointcloud_type " << publishPointCloudType << ", fallback to PointCloud.");
        publishPointCloudType = kSensorPointCloud;
    }
    switch (publishPointCloudType) {
        case kSensorPointCloud:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud>(curr_scan_topic, 5);
            break;
        case kSensorPointCloud2:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);
            break;
#if LIVOX_CUSTOM_MSG_AVAILABLE
        case kLivoxCustomMsg:
            rosPointPub = rosNode->advertise<livox_ros_driver::CustomMsg>(curr_scan_topic, 5);
            break;
#endif
        default:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud>(curr_scan_topic, 5);
            break;
    }

    raySensor = _parent;
    auto sensor_pose = raySensor->Pose();
    SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    // parentEntity = world->GetEntity(_parent->ParentName());
    parentEntity = this->world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);
    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    clip_max_dist_ = maxDist;
    if (sdfPtr->HasElement("max_publish_distance")) {
        clip_max_dist_ = sdfPtr->Get<double>("max_publish_distance");
    }
    if (clip_max_dist_ <= 0.0) {
        clip_max_dist_ = maxDist;
    }
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        const ros::Time current_stamp = ros::Time::now();
        const double first_point_time = points_pair.empty() ? 0.0 : points_pair.front().second.time;
        sensor_msgs::PointCloud scan_point;
        scan_point.header.stamp = current_stamp;
        scan_point.header.frame_id = raySensor->Name();
        auto &scan_points = scan_point.points;
        scan_point.channels.resize(2);
        scan_point.channels[0].name = "intensity";
        scan_point.channels[1].name = "time";
        auto &scan_intensity = scan_point.channels[0].values;
        auto &scan_time = scan_point.channels[1].values;
        std::vector<float> times;
        std::vector<uint16_t> rings;
        times.reserve(points_pair.size());
        rings.reserve(points_pair.size());

#if LIVOX_CUSTOM_MSG_AVAILABLE
        const bool publish_custom_msg = (publishPointCloudType == kLivoxCustomMsg);
        livox_ros_driver::CustomMsg custom_msg;
        if (publish_custom_msg) {
            custom_msg.header.frame_id = scan_point.header.frame_id;
            custom_msg.header.stamp = current_stamp;
            custom_msg.timebase = current_stamp.toNSec();
            custom_msg.lidar_id = 0;
            custom_msg.point_num = 0;
            custom_msg.points.reserve(points_pair.size());
        }
#else
        const bool publish_custom_msg = false;
#endif

        for (auto &pair : points_pair) {
            //int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            //int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            //if (verticle_index < 0 || horizon_index < 0) {
            //   continue;
            //}
            //if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            //   auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= RangeMax() || range <= RangeMin()) {
                    range = 0;
                }
                //scan->set_ranges(index, range);
                //scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();//转换成世界坐标系
                if (range > clip_max_dist_) {
                    continue;
                }

                if (range <= 0.0)
                {
                    continue;
                }

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                scan_points.emplace_back();
                scan_intensity.emplace_back();
                scan_time.emplace_back();
                scan_points.back().x = point.X();
                scan_points.back().y = point.Y();
                scan_points.back().z = point.Z();
                scan_intensity.back() = static_cast<float>(intensity);
                double relative_time = rotate_info.time - first_point_time;
                if (relative_time < 0.0) {
                    relative_time = 0.0;
                }
                constexpr double kTimeScale = 1e-6;  // convert microseconds to seconds
                const float relative_time_f = static_cast<float>(relative_time * kTimeScale);
                scan_time.back() = relative_time_f;
                times.push_back(relative_time_f);
                rings.push_back(static_cast<uint16_t>(rotate_info.line));
#if LIVOX_CUSTOM_MSG_AVAILABLE
                if (publish_custom_msg && range > 0) {
                    livox_ros_driver::CustomPoint pt;
                    pt.x = point.X();
                    pt.y = point.Y();
                    pt.z = point.Z();
                    pt.line = rotate_info.line;
                    pt.tag = 0;
                    double reflectivity = intensity;
                    reflectivity = std::max(0.0, std::min(255.0, reflectivity));
                    pt.reflectivity = static_cast<uint8_t>(reflectivity);
                    double relative_time = rotate_info.time - first_point_time;
                    if (relative_time < 0.0) {
                        relative_time = 0.0;
                    }
                    pt.offset_time = static_cast<uint32_t>(relative_time);
                    custom_msg.points.push_back(pt);
                }
#endif
            //} else {

            //    //                ROS_INFO_STREAM("count is wrong:" << verticle_index << "," << verticalRayCount << ","
            //    //                << horizon_index
            //    //                          << "," << rayCount << "," << pair.second.zenith << "," <<
            //    //                          pair.second.azimuth);
            //}
        }
#if LIVOX_CUSTOM_MSG_AVAILABLE
        if (publish_custom_msg) {
            custom_msg.point_num = static_cast<uint32_t>(custom_msg.points.size());
        }
#endif
        if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);
        switch (publishPointCloudType) {
            case kSensorPointCloud: {
                rosPointPub.publish(scan_point);
                break;
            }
            case kSensorPointCloud2: {
                sensor_msgs::PointCloud2 scan_point2;
                scan_point2.header = scan_point.header;
                sensor_msgs::PointCloud2Modifier modifier(scan_point2);
                modifier.setPointCloud2Fields(
                    6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
                    sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32, "time", 1,
                    sensor_msgs::PointField::FLOAT32, "ring", 1, sensor_msgs::PointField::UINT16);
                modifier.resize(scan_points.size());
                sensor_msgs::PointCloud2Iterator<float> iter_x(scan_point2, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(scan_point2, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(scan_point2, "z");
                sensor_msgs::PointCloud2Iterator<float> iter_intensity(scan_point2, "intensity");
                sensor_msgs::PointCloud2Iterator<float> iter_time(scan_point2, "time");
                sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring(scan_point2, "ring");
                for (size_t idx = 0; idx < scan_points.size();
                     ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_time, ++iter_ring) {
                    const auto& pt = scan_points[idx];
                    *iter_x = pt.x;
                    *iter_y = pt.y;
                    *iter_z = pt.z;
                    *iter_intensity = scan_intensity[idx];
                    *iter_time = times[idx];
                    *iter_ring = rings[idx];
                }
                rosPointPub.publish(scan_point2);
                break;
            }
#if LIVOX_CUSTOM_MSG_AVAILABLE
            case kLivoxCustomMsg: {
                custom_msg.point_num = static_cast<uint32_t>(custom_msg.points.size());
                rosPointPub.publish(custom_msg);
                break;
            }
#endif
            default:
                rosPointPub.publish(scan_point);
                break;
        }
        ros::spinOnce();
    }
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    int ray_index = 0;
    auto ray_size = rays.size();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex; k < end_index; k += downSample) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        if (ray_index < ray_size) {
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, rotate_info);
        }
        ray_index++;
    }
    currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    scan->clear_ranges();
    scan->clear_intensities();

    unsigned int rangeCount = RangeCount();
    unsigned int verticalRangeCount = VerticalRangeCount();

    for (unsigned int j = 0; j < verticalRangeCount; ++j) {
        for (unsigned int i = 0; i < rangeCount; ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}
void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                  const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    // tfBroadcaster->sendTransform(
    //     tf::StampedTransform(tf, ros::Time::now(), father_frame, child_frame));
}

}  // namespace gazebo
