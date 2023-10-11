// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_IMAGE_BUFFER_HPP
#define QML_ROS2_PLUGIN_IMAGE_BUFFER_HPP

#include <QVideoFrame>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace qml_ros2_plugin
{

class ImageBuffer : public QAbstractVideoBuffer
{
public:
  ImageBuffer( sensor_msgs::msg::Image::ConstSharedPtr img,
               const QList<QVideoFrame::PixelFormat> &supported_formats );

  ~ImageBuffer() override;

  MapMode mapMode() const override;

  uchar *map( MapMode, int *num_bytes, int *bytes_per_line ) override;

  void unmap() override;

  QVideoFrame::PixelFormat format() const;

private:
  sensor_msgs::msg::Image::ConstSharedPtr image_;
  QVideoFrame::PixelFormat format_;
  int num_bytes_;
  int bytes_per_line_;
  unsigned char *data_;
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_IMAGE_BUFFER_HPP
