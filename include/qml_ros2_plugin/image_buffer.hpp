/*
 * Copyright (C) 2021  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
