#pragma once

#include "drone_warehouse/warehouse_config.hpp"

#include <QDialog>

class QPushButton;
class QWidget;

class TitleInfoDialog : public QDialog
{
public:
    explicit TitleInfoDialog(QWidget *parent = nullptr);

private:
    void buildUi();//构建UI界面

    QPushButton *parameter_button_ = nullptr;
    QPushButton *close_button_ = nullptr;
};