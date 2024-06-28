#ifndef IMAGE_CONVERSION_HPP
#define IMAGE_CONVERSION_HPP

#include <opencv2/imgproc.hpp>
/*
 *  Copyright (c) [YEAR] [AUTHOR]
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */

namespace image_conversion
{
namespace encodings
{
enum Encoding {
  Unknown,
  RGB,
  RGB555,
  RGB565,
  BGR,
  BGR555,
  BGR565,
  RGBA,
  BGRA,
  MONO,
  YUV422_UYVY,
  YUV422_YUYV,
  NV21,
  NV24
};
}
using Encoding = encodings::Encoding;

namespace impl
{
template<Encoding DEST_ENCODING>
bool convertTo( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding )
{
  return false;
}

template<>
bool convertTo<encodings::RGB>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::RGB565>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::RGB555>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::BGR>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::BGR565>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::BGR555>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::RGBA>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::BGRA>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::MONO>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::YUV422_UYVY>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::YUV422_YUYV>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::NV21>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
// template<>
// bool convertTo<encodings::NV24>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding );
} // namespace impl

bool convert( const cv::Mat &src, Encoding src_encoding, cv::Mat &dest, Encoding dest_encoding )
{
  switch ( dest_encoding ) {
  case encodings::RGB:
    return impl::convertTo<encodings::RGB>( src, dest, src_encoding );
    break;
  case encodings::RGB555:
    return impl::convertTo<encodings::RGB555>( src, dest, src_encoding );
    break;
  case encodings::RGB565:
    return impl::convertTo<encodings::RGB565>( src, dest, src_encoding );
    break;
  case encodings::BGR:
    return impl::convertTo<encodings::BGR>( src, dest, src_encoding );
    break;
  case encodings::BGR555:
    return impl::convertTo<encodings::BGR555>( src, dest, src_encoding );
    break;
  case encodings::BGR565:
    return impl::convertTo<encodings::BGR565>( src, dest, src_encoding );
    break;
  case encodings::RGBA:
    return impl::convertTo<encodings::RGBA>( src, dest, src_encoding );
    break;
  case encodings::BGRA:
    return impl::convertTo<encodings::BGRA>( src, dest, src_encoding );
    break;
  case encodings::MONO:
    return impl::convertTo<encodings::MONO>( src, dest, src_encoding );
    break;
  case encodings::YUV422_UYVY:
    return impl::convertTo<encodings::YUV422_UYVY>( src, dest, src_encoding );
    break;
  }
  return false;
}

namespace impl
{
template<>
bool convertTo<encodings::RGB>( const cv::Mat &src, cv::Mat &dest, Encoding src_encoding )
{
  switch ( src_encoding ) {
  case encodings::RGB:
    src.copyTo( dest );
    return true;
  case encodings::BGR:
    cv::cvtColor( src, dest, cv::COLOR_BGR2RGB );
    return true;
  case encodings::RGBA:
    cv::cvtColor( src, dest, cv::COLOR_RGBA2RGB );
    return true;
  case encodings::BGRA:
    cv::cvtColor( src, dest, cv::COLOR_BGRA2RGB );
    return true;
  case encodings::MONO:
    cv::cvtColor( src, dest, cv::COLOR_GRAY2RGB );
    return true;
  case encodings::YUV422_UYVY:
    cv::cvtColor( src, dest, cv::COLOR_YUV2RGB_UYVY );
    return true;
  case encodings::YUV422_YUYV:
    cv::cvtColor( src, dest, cv::COLOR_YUV2RGB_YUYV );
    return true;
  default:
    return false;
  }
}
} // namespace impl
} // namespace image_conversion

#endif // IMAGE_CONVERSION_HPP
