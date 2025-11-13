/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Adapted for rl_sar to publish Gazebo contact forces as ROS WrenchStamped.
************************************************************************/

#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

namespace gazebo
{
class UnitreeFootContactPlugin : public SensorPlugin
{
   public:
    UnitreeFootContactPlugin() : SensorPlugin() {}
    ~UnitreeFootContactPlugin() override {}

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) override
    {
        this->parent_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
        if (!this->parent_sensor_)
        {
            gzerr << "UnitreeFootContactPlugin requires a ContactSensor.\n";
            return;
        }

        this->ros_node_ = std::make_unique<ros::NodeHandle>("contact");
        const std::string topic = "/visual/" + _sensor->Name() + "/the_force";
        this->force_pub_ = this->ros_node_->advertise<geometry_msgs::WrenchStamped>(topic, 100);

        this->update_connection_ = this->parent_sensor_->ConnectUpdated(
            std::bind(&UnitreeFootContactPlugin::OnUpdate, this));
        this->parent_sensor_->SetActive(true);

        ROS_INFO("Load %s plugin.", _sensor->Name().c_str());
    }

   private:
    void OnUpdate()
    {
        msgs::Contacts contacts = this->parent_sensor_->Contacts();
        const int count = contacts.contact_size();

        double fx = 0.0;
        double fy = 0.0;
        double fz = 0.0;

        for (int i = 0; i < count; ++i)
        {
            const auto& contact = contacts.contact(i);
            const int points = contact.position_size();
            for (int j = 0; j < points; ++j)
            {
                fx += contact.wrench(j).body_1_wrench().force().x();
                fy += contact.wrench(j).body_1_wrench().force().y();
                fz += contact.wrench(j).body_1_wrench().force().z();
            }
            if (points > 0)
            {
                ROS_DEBUG_THROTTLE(2.0,
                                   "[%s] contact between '%s' and '%s', points=%d",
                                   this->parent_sensor_->Name().c_str(),
                                   contact.collision1().c_str(),
                                   contact.collision2().c_str(),
                                   points);
            }
        }

        geometry_msgs::WrenchStamped msg;
        if (count > 0)
        {
            const double denom = static_cast<double>(count);
            msg.wrench.force.x = fx / denom;
            msg.wrench.force.y = fy / denom;
            msg.wrench.force.z = fz / denom;
        }
        else
        {
            msg.wrench.force.x = 0.0;
            msg.wrench.force.y = 0.0;
            msg.wrench.force.z = 0.0;
        }

        this->force_pub_.publish(msg);
    }

    sensors::ContactSensorPtr parent_sensor_;
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Publisher force_pub_;
    event::ConnectionPtr update_connection_;
};

GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}  // namespace gazebo
