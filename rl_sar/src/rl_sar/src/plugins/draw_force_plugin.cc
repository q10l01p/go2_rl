/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
Adapted for rl_sar to visualize contact forces in Gazebo.
************************************************************************/

#include <boost/bind.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include <ignition/math/Color.hh>

namespace gazebo
{
class UnitreeDrawForcePlugin : public VisualPlugin
{
   public:
    UnitreeDrawForcePlugin() : line_(nullptr) {}
    ~UnitreeDrawForcePlugin() override
    {
        if (this->visual_) { this->visual_->DeleteDynamicLine(this->line_); }
    }

    void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) override
    {
        this->visual_ = parent;
        this->visual_namespace_ = "visual/";

        if (sdf->HasElement("topicName"))
        {
            this->topic_name_ = sdf->Get<std::string>("topicName");
        }
        else
        {
            ROS_INFO("Force draw plugin missing <topicName>, defaults to /default_force_draw");
            this->topic_name_ = "/default_force_draw";
        }

        if (sdf->HasElement("scale"))
        {
            this->scale_ = sdf->Get<double>("scale");
            if (this->scale_ <= 0.0)
            {
                ROS_WARN("Force draw plugin <scale> must be positive. Using default 20.0");
                this->scale_ = 20.0;
            }
        }

        if (!ros::isInitialized())
        {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "gazebo_visual",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }

        this->line_ = this->visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
#if GAZEBO_MAJOR_VERSION >= 10
        this->line_->AddPoint(ignition::math::Vector3d::Zero, ignition::math::Color(0, 1, 0, 1.0));
        this->line_->AddPoint(ignition::math::Vector3d::UnitX, ignition::math::Color(0, 1, 0, 1.0));
#else
        this->line_->AddPoint(ignition::math::Vector3d::Zero, common::Color(0, 1, 0, 1.0));
        this->line_->AddPoint(ignition::math::Vector3d::UnitX, common::Color(0, 1, 0, 1.0));
#endif
        this->line_->setMaterial("Gazebo/Purple");
        this->line_->setVisibilityFlags(GZ_VISIBILITY_GUI);
        this->visual_->SetVisible(true);

        this->ros_node_ = std::make_unique<ros::NodeHandle>(this->visual_namespace_);
        this->force_sub_ = this->ros_node_->subscribe(
            this->topic_name_ + "/the_force", 30, &UnitreeDrawForcePlugin::OnForce, this);
        this->update_connection_ =
            event::Events::ConnectPreRender(boost::bind(&UnitreeDrawForcePlugin::OnUpdate, this));

        ROS_INFO("Load %s Draw Force plugin.", this->topic_name_.c_str());
    }

    void OnUpdate()
    {
        this->line_->SetPoint(1, ignition::math::Vector3d(fx_, fy_, fz_));
    }

    void OnForce(const geometry_msgs::WrenchStamped& msg)
    {
        fx_ = msg.wrench.force.x / this->scale_;
        fy_ = msg.wrench.force.y / this->scale_;
        fz_ = msg.wrench.force.z / this->scale_;
    }

   private:
    rendering::VisualPtr visual_;
    rendering::DynamicLines* line_;
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Subscriber force_sub_;
    std::string topic_name_;
    std::string visual_namespace_;
    double fx_{0.0};
    double fy_{0.0};
    double fz_{0.0};
    double scale_{20.0};
    event::ConnectionPtr update_connection_;
};

GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}  // namespace gazebo
