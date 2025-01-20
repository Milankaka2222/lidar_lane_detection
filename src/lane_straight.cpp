#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

class TunnelLaneDetector {
public:
    TunnelLaneDetector() {
        // 订阅激光雷达点云数据
        sub_ = nh_.subscribe("/points_raw", 1, &TunnelLaneDetector::pointCloudCallback, this);
        // 发布车道线信息
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/lane_marker", 1);
        // 发布投影后的点云
        points_2d_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_2d", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 过滤点云，限制在X轴前后十米的范围内
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.0, 10.0);
        pass_x.filter(*cloud_filtered_x);

        // 过滤点云，限制在Y轴前后十米的范围内
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(cloud_filtered_x);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-10.0, 10.0);
        pass_y.filter(*cloud_filtered);

        // 将点云投影到平面（忽略Z轴）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud_filtered->points) {
            if (std::abs(point.z) < 0.1) {  // 假设地面是平面
                pcl::PointXYZ projected_point;
                projected_point.x = point.x;
                projected_point.y = point.y;
                projected_point.z = 0.0;  // 设置Z轴为0，表示投影到2D平面
                cloud_projected->points.push_back(projected_point);
            }
        }

        // 分割点云为左侧和右侧
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud_projected->points) {
            if (point.y > 0) {
                cloud_left->points.push_back(point);
            } else {
                cloud_right->points.push_back(point);
            }
        }

        // 拟合左侧车道线
        pcl::ModelCoefficients::Ptr coefficients_left(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_left(new pcl::PointIndices);
        fitLine(cloud_left, coefficients_left, inliers_left);

        // 拟合右侧车道线
        pcl::ModelCoefficients::Ptr coefficients_right(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_right(new pcl::PointIndices);
        fitLine(cloud_right, coefficients_right, inliers_right);

        // 计算雷达距离左右侧最近的距离
        double distance_left = calculateDistance(coefficients_left);
        double distance_right = calculateDistance(coefficients_right);

        // 计算x=0时的y轴数值
        double y_left_at_x0 = coefficients_left->values[0] * 0.0 + coefficients_left->values[1];
        double y_right_at_x0 = coefficients_right->values[0] * 0.0 + coefficients_right->values[1];

        ROS_INFO("Distance to left lane: %f", distance_left);
        ROS_INFO("Distance to right lane: %f", distance_right);
        // ROS_INFO("y at x=0 for left lane: %f", y_left_at_x0);
        // ROS_INFO("y at x=0 for right lane: %f", y_right_at_x0);
        
        double slope_left = coefficients_left->values[0];
        ROS_INFO("Slope of left lane: %f", slope_left);
        
        double slope_right = coefficients_right->values[0];
        ROS_INFO("Slope of right lane: %f", slope_right);

        // 发布左侧和右侧车道线
        publishLaneMarker(coefficients_left, cloud_msg->header, "left_lane");
        publishLaneMarker(coefficients_right, cloud_msg->header, "right_lane");

        // 计算中间车道线的系数
        pcl::ModelCoefficients::Ptr coefficients_center(new pcl::ModelCoefficients);
        coefficients_center->values.resize(2);
        coefficients_center->values[0] = (coefficients_left->values[0] + coefficients_right->values[0]) / 2.0;  // 平均斜率
        coefficients_center->values[1] = (coefficients_left->values[1] + coefficients_right->values[1]) / 2.0;  // 平均截距

        double slope_center = coefficients_center->values[0];
        ROS_INFO("Slope of center lane: %f", slope_center);

        // 发布中间车道线
        publishLaneMarker(coefficients_center, cloud_msg->header, "center_lane");

        // 发布投影后的点云
        sensor_msgs::PointCloud2 cloud_msg_2d;
        pcl::toROSMsg(*cloud_projected, cloud_msg_2d);
        cloud_msg_2d.header.frame_id = cloud_msg->header.frame_id;
        cloud_msg_2d.header.stamp = ros::Time::now();
        points_2d_pub_.publish(cloud_msg_2d);
    }

private:
    void fitLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.5);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // 检查是否找到内点
        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a line model for the given dataset." << std::endl;
            return;
        }

        // 提取内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*cloud_inliers);

        // 使用最小二乘法进行精确拟合
        if (cloud_inliers->points.size() < 2) {
            std::cerr << "Not enough points to fit a line." << std::endl;
            return;
        }

        Eigen::MatrixXd A(cloud_inliers->points.size(), 2);
        Eigen::VectorXd B(cloud_inliers->points.size());

        for (size_t i = 0; i < cloud_inliers->points.size(); ++i) {
            A(i, 0) = cloud_inliers->points[i].x;
            A(i, 1) = 1.0;
            B(i) = cloud_inliers->points[i].y;
        }

        Eigen::VectorXd X = A.colPivHouseholderQr().solve(B);

        // 更新模型系数
        coefficients->values.resize(2);
        coefficients->values[0] = X(0);  // 斜率 a
        coefficients->values[1] = X(1);  // 截距 b
    }

    double calculateDistance(pcl::ModelCoefficients::Ptr coefficients) {
        // 假设车辆在原点(0, 0, 0)
        double x0 = 0, y0 = 0;
        double a = coefficients->values[0];
        double b = coefficients->values[1];
        return std::abs(b) / std::sqrt(a * a + 1);
    }

    void publishLaneMarker(pcl::ModelCoefficients::Ptr coefficients, const std_msgs::Header& header, const std::string& ns) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;  // 线条宽度
        marker.color.a = 1.0;  // 透明度
        marker.color.r = ns == "left_lane" ? 1.0 : 0.0;  // 左侧车道线为红色
        marker.color.g = ns == "left_lane" ? 0.0 : (ns == "right_lane" ? 1.0 : 0.0);  // 右侧车道线为绿色，中间车道线为蓝色
        marker.color.b = ns == "right_lane" ? 0.0 : (ns == "center_lane" ? 1.0 : 0.0);

        // 计算车道线的起点和终点
        double a = coefficients->values[0];
        double b = coefficients->values[1];

        // 生成更多的点来绘制平滑的曲线
        int num_points = 100;
        for (int i = 0; i < num_points; ++i) {
            double x = 0.0 + (10.0 - (0.0)) * i / (num_points - 1);
            double y = a * x + b;

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0.0;

            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
    ros::Publisher points_2d_pub_;  // 发布投影后的点云
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_lane_detection");
    TunnelLaneDetector detector;
    ros::spin();
    return 0;
}