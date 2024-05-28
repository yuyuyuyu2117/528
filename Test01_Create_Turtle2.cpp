#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");

    // 执行初始化
    ros::init(argc, argv, "create_turtles");
    // 创建节点
    ros::NodeHandle nh;
    // 创建服务客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    // 等待/spawn服务可用
    ros::service::waitForService("/spawn");

    // 创建第一个小海龟
    turtlesim::Spawn spawn1;
    spawn1.request.name = "turtle2";
    spawn1.request.x = 1.0;
    spawn1.request.y = 2.0;
    spawn1.request.theta = 3.12415926;
    bool flag1 = client.call(spawn1);
    if (flag1) {
        ROS_INFO("乌龟%s创建成功!", spawn1.response.name.c_str());
    } else {
        ROS_INFO("乌龟2创建失败!");
    }

    // 创建第二个小海龟
    turtlesim::Spawn spawn2;
    spawn2.request.name = "turtle3";
    spawn2.request.x = 3.0;
    spawn2.request.y = 4.0;
    spawn2.request.theta = 0.0; // 可以根据需要设定方向
    bool flag2 = client.call(spawn2);
    if (flag2) {
        ROS_INFO("乌龟%s创建成功!", spawn2.response.name.c_str());
    } else {
        ROS_INFO("乌龟3创建失败!");
    }

    // 保持节点运行
    ros::spin();

    return 0;
}

