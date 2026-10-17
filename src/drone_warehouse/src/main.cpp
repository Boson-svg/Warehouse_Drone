#include "drone_warehouse/mainwindow.hpp"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    /*********************ros移植部分***********************/
    // 当前程序已经开始接入 RosManager，所以这里需要像原 drone_qt 工程那样先初始化 rclcpp。
    // 否则后面 RosManager 内部创建节点、客户端和订阅者时，ROS 运行时环境并没有准备好。
    rclcpp::init(argc, argv);
    /******************************************************/

    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    const int result = app.exec();

    /*********************ros移植部分***********************/
    // Qt 事件循环退出后，再统一关闭 ROS，和上面的 init 成对出现。
    rclcpp::shutdown();
    /******************************************************/

    return result;
}
