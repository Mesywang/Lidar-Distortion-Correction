#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <champion_nav_msgs/ChampionNavLaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");

class LidarMotionCalibrator
{
public:
    LidarMotionCalibrator(tf::TransformListener *tf)
    {
        tf_ = tf;
        scan_sub_ = nh_.subscribe("champion_scan", 10, &LidarMotionCalibrator::ScanCallBack, this);
    }

    ~LidarMotionCalibrator()
    {
        if (tf_ != NULL)
            delete tf_;
    }

    // 拿到原始的激光数据来进行处理
    void ScanCallBack(const champion_nav_msgs::ChampionNavLaserScanPtr &scan_msg)
    {
        //转换到矫正需要的数据
        ros::Time startTime, endTime;
        startTime = scan_msg->header.stamp;

        champion_nav_msgs::ChampionNavLaserScan laserScanMsg = *scan_msg;

        //得到最终点的时间
        int beamNum = laserScanMsg.ranges.size();
        endTime = startTime + ros::Duration(laserScanMsg.time_increment * (beamNum - 1));

        //将数据复制出来
        std::vector<double> angles, ranges;
        for (int i = beamNum - 1; i > 0; i--)
        {
            double lidar_dist = laserScanMsg.ranges[i];
            double lidar_angle = laserScanMsg.angles[i];

            if (lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
                lidar_dist = 0.0;

            ranges.push_back(lidar_dist);
            angles.push_back(lidar_angle);
        }

        tf::Stamped<tf::Pose> visualPose; // laser在odom系下的位姿
        if (!getLaserPose(visualPose, startTime, tf_))
        {
            ROS_WARN("Not visualPose,Can not Calib");
            return;
        }

        double visualYaw = tf::getYaw(visualPose.getRotation());

        visual_cloud_.clear();
        for (int i = 0; i < ranges.size(); i++)
        {
            if (ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            //将laser_frame下每个激光点的极坐标转换为直角坐标
            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            pcl::PointXYZRGB pt;
            //计算每个激光点在以startTime时位姿为原点的坐标系下的坐标，此处可以理解为里程计模型
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 255, g = 0, b = 0; // 原始点云：红色
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float *>(&rgb);

            visual_cloud_.push_back(pt);
        }

        //进行矫正
        Lidar_Calibration(ranges, angles, startTime, endTime, tf_);

        //转换为pcl::pointcloud for visuailization
        for (int i = 0; i < ranges.size(); i++)
        {
            if (ranges[i] < 0.05 || std::isnan(ranges[i]) || std::isinf(ranges[i]))
                continue;

            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);

            pcl::PointXYZRGB pt;
            pt.x = x * cos(visualYaw) - y * sin(visualYaw) + visualPose.getOrigin().getX();
            pt.y = x * sin(visualYaw) + y * cos(visualYaw) + visualPose.getOrigin().getY();
            pt.z = 1.0;

            unsigned char r = 0, g = 255, b = 0; //矫正后点云：绿色
            unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
            pt.rgb = *reinterpret_cast<float *>(&rgb);

            visual_cloud_.push_back(pt);
        }
        
        g_PointCloudView.showCloud(visual_cloud_.makeShared()); //显示
    }

    /**
     * @name getLaserPose()
     * @brief 得到机器人在里程计坐标系中的位姿tf::Pose
     *        得到dt时刻激光雷达在odom坐标系的位姿
     * @param odom_pos  机器人的位姿
     * @param dt        dt时刻
     * @param tf_
    */
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose,
                      ros::Time dt,
                      tf::TransformListener *tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = "base_laser";
        robot_pose.stamp_ = dt; //设置为ros::Time()表示返回最近的转换关系

        // get the global pose of the robot
        try
        {
            if (!tf_->waitForTransform("/odom", "/base_laser", dt, ros::Duration(0.5)))
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }

        return true;
    }

    /**
     * @brief Lidar_MotionCalibration
     *        激光雷达运动畸变去除分段函数;
     *        在此分段函数中，认为机器人是匀速运动；
     * @param frame_base_pose       标定完毕之后的基准坐标系
     * @param frame_start_pose      本分段第一个激光点对应的位姿
     * @param frame_end_pose        本分段最后一个激光点对应的位姿
     * @param ranges                激光数据－－距离
     * @param angles                激光数据－－角度
     * @param startIndex            本分段第一个激光点在激光帧中的下标
     * @param beam_number           本分段的激光点数量
     */
    void Lidar_MotionCalibration(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double> &ranges,
        std::vector<double> &angles,
        int startIndex,
        int &beam_number)
    {

        //记录插值起点位姿
        double start_x = frame_start_pose.getOrigin().x();
        double start_y = frame_start_pose.getOrigin().y();
        double start_yaw = tf::getYaw(frame_start_pose.getRotation());
        //记录插值终点位姿
        double end_x = frame_end_pose.getOrigin().x();
        double end_y = frame_end_pose.getOrigin().y();
        double end_yaw = tf::getYaw(frame_end_pose.getRotation());

        //每一段的收尾两点会赋值两次
        for (int i = startIndex; i < startIndex + beam_number; i++)
        {
            double x = start_x + ((i - startIndex) / (beam_number - 1)) * (end_x - start_x);
            double y = start_y + ((i - startIndex) / (beam_number - 1)) * (end_y - start_y);
            double yaw = start_yaw + ((i - startIndex) / (beam_number - 1)) * (end_yaw - start_yaw);

            double x_tran = x - frame_base_pose.getOrigin().x();
            double y_tran = y - frame_base_pose.getOrigin().y();
            double yaw_tran = yaw - tf::getYaw(frame_base_pose.getRotation());

            //将laser_frame下每个激光点的极坐标转换为直角坐标
            double laser_x = ranges[i] * cos(angles[i]);
            double laser_y = ranges[i] * sin(angles[i]);

            //将laser_frame下每个激光点转换到以startTime时机器人位姿为原点的坐标系下
            laser_x = laser_x * cos(yaw_tran) - laser_y * sin(yaw_tran) + x_tran;
            laser_y = laser_x * sin(yaw_tran) + laser_y * cos(yaw_tran) + y_tran;

            //将转换后的坐标封装为极坐标形式
            ranges.at(i) = sqrt(pow(laser_x, 2) + pow(laser_y, 2));
            angles.at(i) = atan2(laser_y, laser_x);
        }
    }

    //激光雷达数据　分段线性进行插值　分段的周期为10ms
    //这里会调用Lidar_MotionCalibration()
    /**
     * @name Lidar_Calibration()
     * @brief 激光雷达数据　分段线性进行插值　分段的周期为5ms
     * @param ranges 激光束的距离值集合
     * @param angle　激光束的角度值集合
     * @param startTime　第一束激光的时间戳
     * @param endTime　最后一束激光的时间戳
     * @param *tf_
    */
    void Lidar_Calibration(std::vector<double> &ranges,
                           std::vector<double> &angles,
                           ros::Time startTime,
                           ros::Time endTime,
                           tf::TransformListener *tf_)
    {
        //统计激光束的数量
        int beamNumber = ranges.size();
        if (beamNumber != angles.size())
        {
            ROS_ERROR("Error:ranges not match to the angles");
            return;
        }

        // 5ms来进行分段
        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        //起始时间 us
        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / (beamNumber - 1); // 每束激光数据的时间间隔

        //当前插值的段的起始坐标
        int start_index = 0;

        //起始点的位姿 这里要得到起始点位置的原因是　起始点就是我们的base_pose
        //所有的激光点的基准位姿都会改成我们的base_pose
        // ROS_INFO("get start pose");

        if (!getLaserPose(frame_start_pose, ros::Time(start_time / 1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return;
        }

        if (!getLaserPose(frame_end_pose, ros::Time(end_time / 1000000.0), tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return;
        }

        int cnt = 0;
        //基准坐标就是第一个位姿的坐标
        frame_base_pose = frame_start_pose;
        for (int i = 0; i < beamNumber; i++)
        {
            //分段线性,时间段的大小为interpolation_time_duration
            double mid_time = start_time + time_inc * (i - start_index);
            if (mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;

                if (!getLaserPose(frame_mid_pose, ros::Time(mid_time / 1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error", cnt);
                    return;
                }

                //对当前的起点和终点进行插值
                //interpolation_time_duration中间有多少个点(包括两个端点).
                int interp_count = i - start_index + 1;

                Lidar_MotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,
                                        ranges,
                                        angles,
                                        start_index,
                                        interp_count);

                //更新时间
                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

public:
    tf::TransformListener *tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LidarMotionCalib");

    tf::TransformListener tf(ros::Duration(10.0));

    LidarMotionCalibrator tmpLidarMotionCalib(&tf);

    ros::spin();
    return 0;
}