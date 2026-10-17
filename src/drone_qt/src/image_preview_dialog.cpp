#include "drone_qt/image_preview_dialog.hpp"

#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QScrollArea>
#include <QVBoxLayout>

ImagePreviewDialog::ImagePreviewDialog(QWidget *parent)
    : QDialog(parent)
{
    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);

    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidget(image_label_);
    scroll_area_->setWidgetResizable(true);

    auto *layout = new QVBoxLayout(this);
    layout->addWidget(scroll_area_);

    resize(800, 600);
}

void ImagePreviewDialog::setImage(const QImage &image, const QString &title_text)
{
    setWindowTitle(title_text);
    image_label_->setPixmap(QPixmap::fromImage(image));
    image_label_->adjustSize();
}