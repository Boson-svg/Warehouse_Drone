#pragma once

#include <QString>
#include <QVector>

#include "drone_warehouse/models.hpp"

class ShelfPanelStorage
{
public:
    static QString defaultFilePath();
    static bool save(const QVector<ShelfPanelData> &shelves, QString *error_message = nullptr);
    static bool load(const QString &file_path,
                     QVector<ShelfPanelData> &shelves,
                     QString *error_message = nullptr);
    static bool load(QVector<ShelfPanelData> &shelves,
                     QString *error_message = nullptr);
};
