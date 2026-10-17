#include <QCoreApplication>
#include <QTimer>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>
#include "drone_warehouse/ground_link_bridge.hpp"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    rclcpp::init(argc, argv);

    WarehouseConfig config;
    QString config_error;
    if (!loadWarehouseConfig(config, &config_error)) {
        qCritical().noquote() << "invalid warehouse config:" << config_error;
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<GroundLinkBridge>(
        config.connection.telemetry_serial);

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [node]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    const int rc = app.exec();
    rclcpp::shutdown();
    return rc;
}