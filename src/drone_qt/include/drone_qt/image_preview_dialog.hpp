#pragma once

#include <QDialog>

class QLabel;
class QScrollArea;
class QImage;

class ImagePreviewDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImagePreviewDialog(QWidget *parent = nullptr);
    void setImage(const QImage &image, const QString &title_text);

private:
    QLabel *image_label_{nullptr};
    QScrollArea *scroll_area_{nullptr};
};
