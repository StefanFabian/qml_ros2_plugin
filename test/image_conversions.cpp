// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"
#include "message_comparison.hpp"

#include <qml_ros2_plugin/image_buffer.hpp>
#include <qml_ros2_plugin/ros2.hpp>

#include <QCoreApplication>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

void processSomeEvents( int n = 10, int sleep_duration_us = 5000 )
{
  for ( int i = 0; i < n; ++i ) {
    std::this_thread::sleep_for( std::chrono::microseconds( sleep_duration_us ) );
    QCoreApplication::processEvents();
  }
}

::testing::AssertionResult compareImage( const uint8_t *data, const std::vector<uint8_t> &reference )
{
  for ( size_t i = 0; i < reference.size(); ++i ) {
    if ( data[i] != reference[i] )
      return ::testing::AssertionFailure() << "Image differed at i=" << i << "." << std::endl
                                           << "data[i]: " << static_cast<int>( data[i] ) << std::endl
                                           << "reference[i]: " << static_cast<int>( reference[i] );
  }
  return ::testing::AssertionSuccess();
}

/// ============================================================================
/// =================================== RGB8 ===================================
/// ============================================================================
TEST( ImageConversions, testRGB8 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  image->data = { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGR24, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
    EXPECT_EQ( buffer.mapMode(), QAbstractVideoBuffer::ReadOnly );
    EXPECT_NO_THROW( buffer.unmap() );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_Y8 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, {} );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Invalid );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// =================================== BGR8 ===================================
/// ============================================================================
TEST( ImageConversions, testBGR8 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::BGR8;
  image->data = { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGR24, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32,
                 QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// =================================== RGBA8 ==================================
/// ============================================================================
TEST( ImageConversions, testRGBA8 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 4;
  image->encoding = sensor_msgs::image_encodings::RGBA8;
  image->data = { 255, 0,   0,   128, 0,  255, 0,  128, 200, 100, 0,   255,
                  0,   100, 200, 200, 50, 100, 20, 50,  150, 150, 200, 0 };
  int num_bytes;
  int bytes_per_line;
  {
    // We actually prefer BGRA32 over ARGB32 because on little endian this can be achieved without copy.
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                 QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_RGB24,
                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80ff0000, 0x8000ff00, 0xffc86400,
                             0xc80064c8, 0x32326414, 0x009696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80800000, 0x80008000, 0xffc86400,
                             0xc8004E9C, 0x32091303, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80000080, 0x80008000, 0xff0064c8,
                             0xc89C4E00, 0x32031309, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// =================================== BGRA8 ==================================
/// ============================================================================
TEST( ImageConversions, testBGRA8 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 4;
  image->encoding = sensor_msgs::image_encodings::BGRA8;
  image->data = { 0,   0,   255, 128, 0,  255, 0,  128, 0,   100, 200, 255,
                  200, 100, 0,   200, 20, 100, 50, 50,  200, 150, 150, 0 };
  int num_bytes;
  int bytes_per_line;
  {
    // We actually prefer ARGB32 over BGRA32 because on little endian this can be achieved without copy.
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32_Premultiplied,
                 QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_RGB24,
                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x800000ff, 0x8000ff00, 0xff0064c8,
                             0xc8c86400, 0x32146432, 0x00c89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80000080, 0x80008000, 0xff0064c8,
                             0xc89C4E00, 0x32031309, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80800000, 0x80008000, 0xffc86400,
                             0xc8004E9C, 0x32091303, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// ================================== RGB16 ===================================
/// ============================================================================
TEST( ImageConversions, testRGB16 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 6;
  image->encoding = sensor_msgs::image_encodings::RGB16;
  image->data = { 255, 255, 0, 0,   0, 0,   0, 0,  255, 255, 0, 0,  0, 200, 0, 100, 0, 0,
                  0,   0,   0, 100, 0, 200, 0, 50, 0,   100, 0, 20, 0, 150, 0, 150, 0, 200 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGR24, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C8A, 0x9645, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// ================================== BGR16 ===================================
/// ============================================================================
TEST( ImageConversions, testBGR16 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 6;
  image->encoding = sensor_msgs::image_encodings::BGR16;
  image->data = { 0, 0,   0, 0,   0, 255, 0, 0,  0, 255, 0, 0,  0, 0,   0, 100, 0, 200,
                  0, 200, 0, 100, 0, 0,   0, 20, 0, 100, 0, 50, 0, 200, 0, 150, 0, 150 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGR24, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32,
                 QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
                          QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                          QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// ================================== RGBA16 ==================================
/// ============================================================================
TEST( ImageConversions, testRGBA16 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 8;
  image->encoding = sensor_msgs::image_encodings::RGBA16;
  image->data = { 0, 255, 0, 0,   0, 0,  0, 128, 0, 0,   0, 255, 0, 0,   0, 128,
                  0, 200, 0, 100, 0, 0,  0, 255, 0, 0,   0, 100, 0, 200, 0, 200,
                  0, 50,  0, 100, 0, 20, 0, 50,  0, 150, 0, 150, 0, 200, 0, 0 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80ff0000, 0x8000ff00, 0xffc86400,
                             0xc80064c8, 0x32326414, 0x009696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied,
                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x800000ff, 0x8000ff00, 0xff0064c8,
                             0xc8c86400, 0x32146432, 0x00c89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80800000, 0x80008000, 0xffc86400,
                             0xc8004E9C, 0x32091303, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80000080, 0x80008000, 0xff0064c8,
                             0xc89C4E00, 0x32031309, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// ================================== BGRA16 ==================================
/// ============================================================================
TEST( ImageConversions, testBGRA16 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 8;
  image->encoding = sensor_msgs::image_encodings::BGRA16;
  image->data = { 0, 0,  0, 0,   0, 255, 0, 128, 0, 0,   0, 255, 0, 0,   0, 128,
                  0, 0,  0, 100, 0, 200, 0, 255, 0, 200, 0, 100, 0, 0,   0, 200,
                  0, 20, 0, 100, 0, 50,  0, 50,  0, 200, 0, 150, 0, 150, 0, 0 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
                                 QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x800000ff, 0x8000ff00, 0xff0064c8,
                             0xc8c86400, 0x32146432, 0x00c89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32_Premultiplied,
                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80ff0000, 0x8000ff00, 0xffc86400,
                             0xc80064c8, 0x32326414, 0x009696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80000080, 0x80008000, 0xff0064c8,
                             0xc89C4E00, 0x32031309, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0x80800000, 0x80008000, 0xffc86400,
                             0xc8004E9C, 0x32091303, 0x00000000 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 0, 0, 255, 0, 255, 0, 0, 100, 200, 200, 100, 0, 20, 100, 50, 200, 150, 150 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
                                 QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 255, 0, 0, 0, 255, 0, 200, 100, 0, 0, 100, 200, 50, 100, 20, 150, 150, 200 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff0000ff, 0xff00ff00, 0xff0064c8,
                             0xffc86400, 0xff146432, 0xffc89696 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xffff0000, 0xff00ff00, 0xffc86400,
                             0xff0064c8, 0xff326414, 0xff9696c8 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 76, 149, 118, 81, 75, 155 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x4C3E, 0x95AF, 0x7680, 0x5180, 0x4BEE, 0x9BB3 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
}

/// ============================================================================
/// =================================== MONO8 ==================================
/// ============================================================================
TEST( ImageConversions, testMONO8 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2;
  image->encoding = sensor_msgs::image_encodings::MONO8;
  image->data = { 85, 0, 100, 255, 56, 166 };
  int num_bytes;
  int bytes_per_line;
  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                 QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_EQ( data, image->data.data() ) << "This shouldn't require a copy!";
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x5500, 0x0000, 0x6400, 0xff00, 0x3800, 0xA600 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }
}

/// ============================================================================
/// ================================== MONO16 ==================================
/// ============================================================================
TEST( ImageConversions, testMONO16 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 2;
  image->encoding = sensor_msgs::image_encodings::MONO16;
  image->is_bigendian = 1;
  image->data = { 85, 0, 0, 0, 100, 0, 255, 0, 56, 170, 166, 105 };
  int num_bytes;
  int bytes_per_line;

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                 QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x5500, 0x0000, 0x6400, 0xff00, 0x38AA, 0xA669 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 85, 0, 100, 255, 56, 166 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }
}

/// ============================================================================
/// ================================== 32FC1 ===================================
/// ============================================================================
TEST( ImageConversions, test32FC1 )
{
  sensor_msgs::msg::Image::SharedPtr image = std::make_shared<sensor_msgs::msg::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 4;
  image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  float img_data[] = { 0.333334f, 0.0f, 0.392157f, 1.0f, 0.221348f, 0.6509804f };
  auto *img_data_ptr = reinterpret_cast<uint8_t *>( img_data );
  image->data.assign( img_data_ptr, img_data_ptr + 24 );
  int num_bytes;
  int bytes_per_line;

  {
    ImageBuffer buffer(
        image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                 QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
                 QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y16 );
    EXPECT_EQ( num_bytes, 2 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 2 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint16_t reference[] = { 0x5555, 0x0000, 0x6464, 0xffff, 0x38AA, 0xA6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 12 ) ) );
  }
  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
                                 QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_Y8 );
    EXPECT_EQ( num_bytes, 2 * 3 );
    EXPECT_EQ( bytes_per_line, 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage( data, { 85, 0, 100, 255, 56, 166 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                                 QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB24,
                                 QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB24 );
    EXPECT_EQ( num_bytes, 3 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 3 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    EXPECT_TRUE( compareImage(
        data, { 85, 85, 85, 0, 0, 0, 100, 100, 100, 255, 255, 255, 56, 56, 56, 166, 166, 166 } ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGR32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGR32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image,
                        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
                          QVideoFrame::Format_BGRA32_Premultiplied,
                          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_RGB32 } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_RGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32_Premultiplied,
                                 QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32 );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_BGRA32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_BGRA32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }

  {
    ImageBuffer buffer( image, { QVideoFrame::Format_ARGB32_Premultiplied } );
    uchar *data = buffer.map( QAbstractVideoBuffer::ReadOnly, &num_bytes, &bytes_per_line );
    ASSERT_EQ( buffer.format(), QVideoFrame::Format_ARGB32_Premultiplied );
    EXPECT_EQ( num_bytes, 4 * 3 * 2 );
    EXPECT_EQ( bytes_per_line, 4 * 2 );
    EXPECT_NE( data, image->data.data() ) << "Copy is necessary!";
    uint32_t reference[] = { 0xff555555, 0xff000000, 0xff646464,
                             0xffffffff, 0xff383838, 0xffA6A6A6 };
    auto *ref_ptr = reinterpret_cast<uint8_t *>( reference );
    EXPECT_TRUE( compareImage( data, std::vector<uint8_t>( ref_ptr, ref_ptr + 24 ) ) );
  }
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}