#pragma once

#include <QColor>

namespace ColorPalette
{
    inline const QColor Black        = QColor(0, 0, 0);//黑色
    inline const QColor White        = QColor(255, 255, 255);//白色
    inline const QColor BlueBlack    = QColor(12, 16, 24);//蓝黑色
    inline const QColor BlueGrayDark = QColor(45, 60, 80);//蓝灰色（深）

    inline const QColor GrayDark     = QColor(40, 40, 40);//深灰色
    inline const QColor Gray         = QColor(128, 128, 128);//灰色
    inline const QColor GrayLight    = QColor(200, 200, 200);//浅灰色

    inline const QColor Red          = QColor(255, 0, 0);//红色
    inline const QColor RedDark      = QColor(180, 40, 40);//深红色
    inline const QColor Orange       = QColor(255, 140, 0);//橙色
    inline const QColor Yellow       = QColor(255, 215, 0);//黄色

    inline const QColor Green        = QColor(0, 180, 90);//绿色
    inline const QColor GreenLight   = QColor(80, 220, 140);//浅绿色
    inline const QColor AquaGreen    = QColor(0, 255, 180);//荧光绿

    inline const QColor Blue         = QColor(0, 120, 255);//蓝色
    inline const QColor BlueDark     = QColor(40, 70, 110);//深蓝色
    inline const QColor BlueLight    = QColor(120, 180, 255);//浅蓝色

    inline const QColor Cyan         = QColor(0, 220, 255);//青色
    inline const QColor CyanLight    = QColor(120, 240, 255);//浅青色
    inline const QColor CyanBlue     = QColor(0, 150, 200);//青蓝色

    inline const QColor Purple       = QColor(140, 90, 220);//紫色
    inline const QColor PurpleLight  = QColor(190, 150, 255);//浅紫色

    inline const QColor Pink         = QColor(255, 120, 180);//粉色
    inline const QColor Brown        = QColor(120, 80, 50);//棕色

    inline const QColor Transparent  = QColor(0, 0, 0, 0);//完全透明

    inline QColor withAlpha(const QColor &color, int alpha)
    {
        QColor c = color;
        c.setAlpha(alpha);
        return c;
    }
}