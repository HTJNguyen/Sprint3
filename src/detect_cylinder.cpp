#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h> // For quaternion to Euler conversion
#include <tf2/LinearMath/Matrix3x3.h>  // For quaternion to Euler conversion
#include <cmath>
#include <vector>
#include <numeric>

class DetectCylinderNode : public rclcpp::Node
{
public:
    DetectCylinderNode() : Node("detect_cylinder_node")
    {
        // Declare parameters for the cylinder's position and radius
        this->declare_parameter<float>("cylinder_radius", 0.15); // 15 cm radius for a 30 cm diameter
        this->declare_parameter<float>("cylinder_x", 3.0);       // Default x position
        this->declare_parameter<float>("cylinder_y", 0.0);       // Default y position
        this->declare_parameter<float>("cylinder_z", 0.0);       // Default z position

        // Get parameters
        this->get_parameter("cylinder_radius", cylinderRadius_);
        this->get_parameter("cylinder_x", cylinderPosition_.x);
        this->get_parameter("cylinder_y", cylinderPosition_.y);
        this->get_parameter("cylinder_z", cylinderPosition_.z);

        // Subscribe to laser scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DetectCylinderNode ::laserCallback, this, std::placeholders::_1));

        // Subscribe to odom topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DetectCylinderNode ::odomCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        cylinder_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_marker", 10);

        // Publisher for detection markers
        detection_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/detection_marker", 10);

        // Publish the marker to represent the cylinder in the environment
        publishCylinderMarker();
    }

private:
    /**
     * @brief Callback function for the LaserScan subscriber.
     *
     * Saves the laserScan to a local copy if odometry has been recieved.
     * This is done to ensure that the laserscan and the odometry information are synced up.
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // If no odometry information is saved then don't continue.
        if (!recievedOdometry_)
        {
            return;
        }

        // Save a local copy of the LaserScan.
        laserScan_ = *scan;

        // Filter out bad scans.
        filterScan();
        // Find "objects" which may be the cylinder.
        detectObjects();
        // Detect the Cylinder from the found objects.
        detectCylinder();
    }

    /**
     * @brief Callback function for the Odometry subscriber.
     *
     * Saves the odometry to a local copy if a local copy doesn't exist.
     * This is done to ensure that the laserscan and the odometry information are synced up.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // If a local copy exists then don't update it.
        if (recievedOdometry_)
        {
            return;
        }

        // Save a local copy of the Odometry and set recievedOdometry_ to true.
        odometry_.position = odom->pose.pose.position;
        odometry_.orientation = odom->pose.pose.orientation;
        recievedOdometry_ = true;
    }

    /**
     * Filter the scan. It should:
     * - Be Finite.
     * - Be a number.
     * - Be less than the max range amount.
     * - Be greater than the min range amount.
     *
     * @returns Valid points in the validRanges_ vector.
     */
    void filterScan()
    {
        RCLCPP_INFO(this->get_logger(), "REACHED filterScan()");
        // Clear the previous scan.
        validRanges_.clear();
        for (float scan : laserScan_.ranges)
        {
            if (!isnan(scan) && isfinite(scan) && scan < laserScan_.range_max && scan > laserScan_.range_min)
            {
                validRanges_.push_back(scan);
            }
        }
        RCLCPP_INFO(this->get_logger(), "FINISHED filterScan()");

        // Find "objects" which may be the cylinder.
        // detectObjects();
    }

    /**
     * @brief Look through the valid points for valid objects.
     *
     * Valid objects are a set of points that are connected with a distance within the tolerance range.
     */
    void detectObjects()
    {
        RCLCPP_INFO(this->get_logger(), "REACHED detectObjects()");
        geometry_msgs::msg::Point global1, global2, local1, local2;

        geometry_msgs::msg::Pose odo = getOdometry();

        double yaw = getYawFromPose(odo);               //!< Converted yaw from provided pose.
        std::vector<geometry_msgs::msg::Point> objects; //!< Points that make up objects which may be the cylinder.

        int state = 0; // Initialize state

        for (int i = 1; i < validRanges_.size(); i++)
        {
            // Convert local coordinates from Polar to Cartesian.
            local1 = polarToCartesian(validRanges_.at(i - 1), yaw);
            local2 = polarToCartesian(validRanges_.at(i), yaw);

            // Convert local coordinates into global coordinates.
            global1.x = odo.position.x + local1.x;
            global1.y = odo.position.y + local1.y;
            global1.z = 0;

            global2.x = odo.position.x + local2.x;
            global2.y = odo.position.y + local2.y;
            global2.z = 0;

            // Calculate the distance between the two points.
            float distance = std::hypot(global2.x - global1.x, global2.y - global1.y);

            // Switch statement to handle different states.
            if (state == 0) // State 0: The vector is empty, add the first point.
            {
                objects.push_back(global1); // Add global1 point to objects.
                state = 1; // Set state.
            }
            if (distance < 0.3) // Check if the distance from global1 to global2 is withing the tolerance.
            {
                objects.push_back(global2); // Add global2 point to objects.
                state = 2;
            }
            else
            {
                if (state == 1) // State 1: Distance was greater than the tolerance, clear the vector.
                {
                    objects.clear(); // Clear the objects vector.
                    state = 0;
                }
                if (state == 2)
                {
                    detectedObjects_.push_back(objects); // State 2: Add the objects vector to the detectedObjects_ vector and restart the search.
                    objects.clear();
                    state = 0;
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "FINISHED detectObjects()");

        // Detect the Cylinder from the found objects.
        // detectCylinder();
    }

    /**
     * @brief Detects the Cylinder from the vector of found objects.
     *
     * Tolerances for what is considered a cylinder are set by the user. (Magic numbers).
     */
    void detectCylinder()
    {
        RCLCPP_INFO(this->get_logger(), "REACHED detectCylinder()");
        cylinderDetected_ = false;

        // Look through the detected objects vector.
        for (auto &object : detectedObjects_)
        {
            // If the object size is within the tolerance, then it could be a cylinder.
            if (object.size() <= 7 && object.size() >= 3)
            {
                double distance = std::hypot(object.back().x - object.front().x, object.back().y - object.front().y);
                // If the distance between the two points is within tolerance.
                if (fabs(distance) <= 0.5)
                {
                    // Cylinder has been found.
                    cylinderDetected_ = true;

                    // Find midpoint between checked points to place marker.
                    float x = (object.front().x + object.back().x) / 2;
                    float y = (object.front().y + object.back().y) / 2;

                    // Log the found x and y.
                    RCLCPP_INFO(this->get_logger(), "   FOUND CYLINDER AT x = %f, y = %f", x, y);

                    // Publish the detection.
                    publishDetectionMarker(x, y);

                    break;
                }
            }
        }
        detectedObjects_.clear();

        RCLCPP_INFO(this->get_logger(), "FINISHED detectCylinder()");

        recievedOdometry_ = false;
    }

    /**
     * @brief Convert the polar coordinates to cartesian coordinates.
     *
     * @param range The range scan.
     * @param yaw The angle of the robot.
     *
     * @returns The coverted coordinates as a Point.
     */
    geometry_msgs::msg::Point polarToCartesian(double range, double yaw)
    {
        geometry_msgs::msg::Point point;
        point.x = range * cos(yaw);
        point.y = range * sin(yaw);
        point.z = 0.0; // Not needed.

        return point;
    }

    /**
     * @brief Gets the Yaw from the Pose of the robot.
     *
     * @param pose The pose of the robot. Retrieved from the Odometry data.
     *
     * @returns The converted Yaw.
     */
    double getYawFromPose(const geometry_msgs::msg::Pose &pose)
    {
        // Extract the quaternion from the pose.
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        // Convert quaternion to yaw (Z-axis rotation)
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }

    /**
     * @brief Gets the Odometry from the locally stored variable.
     *
     * @returns Odometry
     */
    geometry_msgs::msg::Pose getOdometry()
    {
        return odometry_;
    }

    // Function to publish the marker representing the cylindrical object
    void publishCylinderMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker pose to the user-specified position
        marker.pose.position = cylinderPosition_;
        marker.pose.orientation.w = 1.0; // No rotation

        // Set scale (30cm diameter cylinder)
        marker.scale.x = 0.30;
        marker.scale.y = 0.30;
        marker.scale.z = 1.0; // Cylinder height

        // Set color (blue cylinder)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0); // Permanent marker

        // Publish the marker
        cylinder_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Cylinder marker published at x = %f, y = %f, z = %f",
                    cylinderPosition_.x, cylinderPosition_.y, cylinderPosition_.z);
    }

    // Function to publish the marker representing detections
    void publishDetectionMarker(float detected_x, float detected_y)
    {
        visualization_msgs::msg::Marker detection_marker;
        detection_marker.header.frame_id = "base_link";
        detection_marker.header.stamp = this->get_clock()->now();
        detection_marker.ns = "detection";
        detection_marker.id = 1;                                         // Unique ID for the detection marker
        detection_marker.type = visualization_msgs::msg::Marker::SPHERE; // Use a sphere to represent the detection
        detection_marker.action = visualization_msgs::msg::Marker::ADD;

        // Set marker pose to the detected position
        detection_marker.pose.position.x = detected_x;
        detection_marker.pose.position.y = detected_y;
        detection_marker.pose.position.z = 0.0;    // Set to ground level or appropriate height
        detection_marker.pose.orientation.w = 1.0; // No rotation

        // Set scale (20 cm diameter sphere, which is a radius of 10 cm)
        detection_marker.scale.x = 0.20; // Diameter
        detection_marker.scale.y = 0.20; // Diameter
        detection_marker.scale.z = 0.20; // Height

        // Set color (green for detection)
        detection_marker.color.r = 0.0;
        detection_marker.color.g = 1.0;
        detection_marker.color.b = 0.0;
        detection_marker.color.a = 1.0; // Fully opaque

        detection_marker.lifetime = rclcpp::Duration::from_seconds(0); // Permanent marker

        // Publish the detection marker
        detection_pub_->publish(detection_marker);

        RCLCPP_INFO(this->get_logger(), "Detection marker published at x = %f, y = %f",
                    detected_x, detected_y);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;      //!< LaserScan subscriber.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;           //!< Odometry subscriber.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cylinder_pub_;  //!< Cylinder marker publisher.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_pub_; //!< Detection marker publisher.

    sensor_msgs::msg::LaserScan laserScan_;                               //!< [sensor_msgs::msg::LaserScan] Local copy of laserscan.
    geometry_msgs::msg::Pose odometry_;                                   //!< [geometry_msgs::msg::Pose] Local copy of Odometry.
    std::vector<float> validRanges_;                                      //!< [std::vector<float>] A vector of valid ranges from the scan data.
    std::vector<std::vector<geometry_msgs::msg::Point>> detectedObjects_; //<! [std::vector<std::vector<geometry_msgs::msg::Point>>] A vector containing another vector which contains points which make up an "object" that could be the cylinder.

    bool recievedOdometry_ = false; //!< [bool] Indicates if the odometry information has been recieved.
    bool cylinderDetected_ = false; //!< [bool] Indicates if the cylinder has been detected.

    float cylinderRadius_;
    geometry_msgs::msg::Point cylinderPosition_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}