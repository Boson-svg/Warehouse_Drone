#include <QApplication>
#include <thread>

#include "drone_qt/mainwindow.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    //创建主窗口对象，并显示界面
    MainWindow window;
    window.show();

    //进入Qt事件循环，等待用户操作
    const int ret = app.exec();

    //退出Qt事件循环后，关闭ROS spinning，并等待线程结束
    rclcpp::shutdown();
    return ret;
}