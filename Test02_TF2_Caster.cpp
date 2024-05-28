#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

// 保存乌龟名称
std::string turtle_name;

// 回调函数，处理乌龟的位置信息并发布TF变换
void doPose(const turtlesim::Pose::ConstPtr& pose) {
    // 创建 TF 广播器
    static tf2_ros::TransformBroadcaster broadcaster;
    // 将 pose 信息转换成 TransFormStamped
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = "world";
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = turtle_name;
    tfs.transform.translation.x = pose->x;
    tfs.transform.translation.y = pose->y;
    tfs.transform.translation.z = 0.0;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
    // 发布
    broadcaster.sendTransform(tfs);
} 

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    // 初始化 ros 节点
    ros::init(argc, argv, "follow_turtles_tf");
    // 解析传入的命名空间
    if (argc != 2) {
        ROS_ERROR("请传入正确的参数");
        return 1;
    } else {
        turtle_name = argv[1];
        ROS_INFO("乌龟 %s 坐标发送启动",turtle_name.c_str());
    }

    // 创建 ros 句柄
    ros::NodeHandle nh;
    // 创建订阅对象
    ros::Subscriber sub = nh.subscribe<turtlesim::Pose>(turtle_name + "/pose", 1000, doPose);
    // spin
    ros::spin();
    return 0;
}

