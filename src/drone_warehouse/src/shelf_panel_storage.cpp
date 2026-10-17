#include "drone_warehouse/shelf_panel_storage.hpp"

#include "drone_warehouse/warehouse_config.hpp"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

namespace
{
    QJsonObject slotImageToJson(const SlotImageData &image)
    {
        QJsonObject object;
        object.insert("image_format", image.image_format);
        object.insert("barcode", image.barcode);
        object.insert("time_text", image.time_text);
        object.insert("image_data_base64", QString::fromLatin1(image.image_data.toBase64()));
        return object;
    }

    QJsonObject shelfSlotToJson(const ShelfSlotItem &slot)
    {
        QJsonObject object;
        object.insert("category_id", slot.category_id);
        object.insert("package_id", slot.package_id);
        object.insert("observed_category_id", slot.observed_category_id);
        object.insert("observed_package_id", slot.observed_package_id);
        object.insert("position_package_id", slot.position_package_id);
        object.insert("observed_time_text", slot.observed_time_text);
        object.insert("has_image", slot.has_image);
        object.insert("latest_image", slotImageToJson(slot.latest_image));
        return object;
    }

    QJsonObject shelfPanelToJson(const ShelfPanelData &shelf)
    {
        QJsonArray front_slots;
        for (const ShelfSlotItem &slot : shelf.front_slots)
        {
            front_slots.append(shelfSlotToJson(slot));
        }

        QJsonArray back_slots;
        for (const ShelfSlotItem &slot : shelf.back_slots)
        {
            back_slots.append(shelfSlotToJson(slot));
        }

        QJsonObject object;
        object.insert("front_slots", front_slots);
        object.insert("back_slots", back_slots);
        return object;
    }

    SlotImageData slotImageFromJson(const QJsonObject &object)
    {
        SlotImageData image;
        image.image_format = object.value("image_format").toString();
        image.barcode = object.value("barcode").toString();
        image.time_text = object.value("time_text").toString();
        image.image_data = QByteArray::fromBase64(object.value("image_data_base64").toString().toLatin1());
        return image;
    }

    ShelfSlotItem shelfSlotFromJson(const QJsonObject &object)
    {
        ShelfSlotItem slot;
        slot.category_id = object.value("category_id").toString();
        slot.package_id = object.value("package_id").toString();
        slot.observed_category_id = object.value("observed_category_id").toString();
        slot.observed_package_id = object.value("observed_package_id").toString();
        slot.position_package_id = object.value("position_package_id").toString();
        slot.observed_time_text = object.value("observed_time_text").toString();
        slot.has_image = object.value("has_image").toBool(false);
        slot.latest_image = slotImageFromJson(object.value("latest_image").toObject());
        if (!slot.has_image || slot.latest_image.image_data.isEmpty())
        {
            slot.has_image = false;
            slot.latest_image = SlotImageData{};
        }
        return slot;
    }

    bool fillShelfPanelFromJson(const QJsonObject &object,
                                int expected_slots_per_side,
                                ShelfPanelData &shelf)
    {
        const QJsonArray front_slots = object.value("front_slots").toArray();
        const QJsonArray back_slots = object.value("back_slots").toArray();
        if (front_slots.size() != expected_slots_per_side ||
            back_slots.size() != expected_slots_per_side)
        {
            return false;
        }
        shelf.front_slots.clear();
        shelf.back_slots.clear();
        shelf.front_slots.reserve(front_slots.size());
        shelf.back_slots.reserve(back_slots.size());

        for (const QJsonValue &value : front_slots)
        {
            if (!value.isObject())
            {
                return false;
            }
            shelf.front_slots.push_back(shelfSlotFromJson(value.toObject()));
        }

        for (const QJsonValue &value : back_slots)
        {
            if (!value.isObject())
            {
                return false;
            }
            shelf.back_slots.push_back(shelfSlotFromJson(value.toObject()));
        }

        return true;
    }
}

QString ShelfPanelStorage::defaultFilePath()
{
    QDir data_directory(warehouseDataDirectory());
    if (!data_directory.exists())
    {
        data_directory.mkpath(".");
    }
    return shelfPanelDataFilePath();
}

bool ShelfPanelStorage::save(const QVector<ShelfPanelData> &shelves, QString *error_message)
{
    QJsonArray shelves_array;
    for (const ShelfPanelData &shelf : shelves)
    {
        shelves_array.append(shelfPanelToJson(shelf));
    }

    QJsonObject root;
    root.insert("version", 1);
    root.insert("shelves", shelves_array);

    QSaveFile file(defaultFilePath());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (error_message)
        {
            *error_message = QString("无法打开文件 %1").arg(file.fileName());
        }
        return false;
    }

    const QByteArray json_bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(json_bytes) != json_bytes.size())
    {
        if (error_message)
        {
            *error_message = QString("写入不完整 %1").arg(file.fileName());
        }
        file.cancelWriting();
        return false;
    }

    if (!file.commit())
    {
        if (error_message)
        {
            *error_message = QString("提交文件失败 %1").arg(file.fileName());
        }
        return false;
    }

    return true;
}

bool ShelfPanelStorage::load(const QString &file_path,
                             QVector<ShelfPanelData> &shelves,
                             QString *error_message)
{
    QFile file(file_path);
    if (!file.exists())
    {
        if (error_message)
        {
            *error_message = QString("文件不存在 %1").arg(file.fileName());
        }
        return false;
    }

    if (!file.open(QIODevice::ReadOnly))
    {
        if (error_message)
        {
            *error_message = QString("无法读取文件 %1").arg(file.fileName());
        }
        return false;
    }

    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !doc.isObject())
    {
        if (error_message)
        {
            *error_message = QString("JSON 解析错误 %1").arg(parse_error.errorString());
        }
        return false;
    }

    const QJsonArray shelves_array = doc.object().value("shelves").toArray();
    if (shelves_array.isEmpty())
    {
        if (error_message)
        {
            *error_message = "shelves 为空";
        }
        return false;
    }

    if (shelves_array.size() != shelves.size())
    {
        if (error_message)
        {
            *error_message = QString("货架数量不匹配：配置为 %1，文件为 %2")
                .arg(shelves.size())
                .arg(shelves_array.size());
        }
        return false;
    }

    QVector<ShelfPanelData> loaded_shelves;
    loaded_shelves.reserve(shelves_array.size());
    for (int shelf_index = 0; shelf_index < shelves_array.size(); ++shelf_index)
    {
        const QJsonValue value = shelves_array.at(shelf_index);
        if (!value.isObject())
        {
            if (error_message)
            {
                *error_message = "存在非对象货架项";
            }
            return false;
        }

        ShelfPanelData shelf = shelves.at(shelf_index);
        const int expected_slots_per_side = shelf.front_slots.size();
        if (expected_slots_per_side <= 0 ||
            shelf.back_slots.size() != expected_slots_per_side ||
            !fillShelfPanelFromJson(
                value.toObject(), expected_slots_per_side, shelf))
        {
            if (error_message)
            {
                *error_message = "货架槽位数量不合法或槽位对象无效";
            }
            return false;
        }

        loaded_shelves.push_back(shelf);
    }

    shelves = loaded_shelves;
    return true;
}

bool ShelfPanelStorage::load(QVector<ShelfPanelData> &shelves,
                             QString *error_message)
{
    return load(defaultFilePath(), shelves, error_message);
}