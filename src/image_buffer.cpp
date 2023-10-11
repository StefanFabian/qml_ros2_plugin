// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/image_buffer.hpp"

#if ( defined( _MSVC_LANG ) && _MSVC_LANG >= 202002L ) || __cplusplus >= 202002L
  #include <bit>
namespace qml_ros2_plugin
{
namespace
{
using endian = std::endian;
} // namespace
} // namespace qml_ros2_plugin
#else
namespace qml_ros2_plugin
{
namespace
{
enum endian {
  little = 0,
  big = 1,
  #ifndef BYTEORDER_ENDIAN
    // Detect with GCC 4.6's macro.
    #if defined( __BYTE_ORDER__ )
      #if ( __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ )
  native = little
      #elif ( __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__ )
  native = big
      #else
        #error "Unknown machine byteorder endianness detected. User needs to define BYTEORDER_ENDIAN."
      #endif
    // Detect with GLIBC's endian.h.
    #elif defined( __GLIBC__ )
      #include <endian.h>
      #if ( __BYTE_ORDER == __LITTLE_ENDIAN )
  native = little
      #elif ( __BYTE_ORDER == __BIG_ENDIAN )
  native = big
      #else
        #error "Unknown machine byteorder endianness detected. User needs to define BYTEORDER_ENDIAN."
      #endif
    // Detect with _LITTLE_ENDIAN and _BIG_ENDIAN macro.
    #elif defined( _LITTLE_ENDIAN ) && !defined( _BIG_ENDIAN )
  native = little
    #elif defined( _BIG_ENDIAN ) && !defined( _LITTLE_ENDIAN )
  native = big
    // Detect with architecture macros.
    #elif defined( __sparc ) || defined( __sparc__ ) || defined( _POWER ) ||                       \
        defined( __powerpc__ ) || defined( __ppc__ ) || defined( __hpux ) || defined( __hppa ) ||  \
        defined( _MIPSEB ) || defined( _POWER ) || defined( __s390__ )
  native = big
    #elif defined( __i386__ ) || defined( __alpha__ ) || defined( __ia64 ) ||                      \
        defined( __ia64__ ) || defined( _M_IX86 ) || defined( _M_IA64 ) || defined( _M_ALPHA ) ||  \
        defined( __amd64 ) || defined( __amd64__ ) || defined( _M_AMD64 ) ||                       \
        defined( __x86_64 ) || defined( __x86_64__ ) || defined( _M_X64 ) || defined( __bfin__ )
  native = little
    #elif defined( _MSC_VER ) && ( defined( _M_ARM ) || defined( _M_ARM64 ) )
  native = little
    #else
      #error "Unknown machine byteorder endianness detected. User needs to define BYTEORDER_ENDIAN."
    #endif
  #endif
};
} // namespace
} // namespace qml_ros2_plugin
#endif

namespace qml_ros2_plugin
{

namespace
{

template<typename T>
T byteSwap( T val );

template<>
uint8_t byteSwap( uint8_t val )
{
  return val;
}

#ifdef __GNUC__

// For GCC use builtin swap
template<>
uint16_t byteSwap( uint16_t val )
{
  return __builtin_bswap16( val );
}

#else

template<>
uint16_t byteSwap( uint16_t val )
{
  return ( val << 8U ) | ( val >> 8U );
}

#endif

template<typename T>
T returnMax( const uint8_t *, T max )
{
  return max;
}

template<typename T, int channel, bool swapEndianness = false>
T extractChannel( const uint8_t *stream, T max )
{
  T val = *( reinterpret_cast<const T *>( stream ) + channel );
  if ( swapEndianness )
    val = byteSwap( val );
  if ( max >= std::numeric_limits<T>::max() )
    return val * ( ( max + 1 ) / ( std::numeric_limits<T>::max() + 1 ) );
  return val / ( ( std::numeric_limits<T>::max() + 1 ) / ( max + 1 ) );
}

template<int channel>
float extractFloatChannelFromBigEndian( const uint8_t *stream, float max )
{
  stream += 4 * channel;
  float value = 0;
  void *data = &value;
  *reinterpret_cast<uint32_t *>( data ) = ( static_cast<uint32_t>( *stream ) << 24U ) |
                                          ( static_cast<uint32_t>( *( stream + 1 ) ) << 16U ) |
                                          ( static_cast<uint32_t>( *( stream + 2 ) ) << 8U ) |
                                          ( static_cast<uint32_t>( *( stream + 3 ) ) );
  return value * max;
}

template<int channel>
float extractFloatChannelFromLittleEndian( const uint8_t *stream, float max )
{
  stream += 4 * channel;
  float value = 0;
  void *data = &value;
  *reinterpret_cast<uint32_t *>( data ) = ( static_cast<uint32_t>( *( stream + 3 ) ) << 24U ) |
                                          ( static_cast<uint32_t>( *( stream + 2 ) ) << 16U ) |
                                          ( static_cast<uint32_t>( *( stream + 1 ) ) << 8U ) |
                                          ( static_cast<uint32_t>( *stream ) );
  return value * max;
}

template<typename T>
using Extractor = T( const uint8_t *, T );

template<typename T>
void setFromStream( uint8_t *&output, T val )
{
  *output = val;
  ++output;
}

template<typename T, Extractor<T> C1, Extractor<T> C2, Extractor<T> C3>
void setPixel( const uint8_t *stream, uint8_t *&output )
{
  setFromStream( output, static_cast<uint8_t>( C1( stream, 255 ) ) );
  setFromStream( output, static_cast<uint8_t>( C2( stream, 255 ) ) );
  setFromStream( output, static_cast<uint8_t>( C3( stream, 255 ) ) );
}

template<typename T, Extractor<T> C1, Extractor<T> C2, Extractor<T> C3, Extractor<T> C4>
void setPixel( const uint8_t *stream, uint32_t *&output )
{
  *output = ( static_cast<uint32_t>( static_cast<uint8_t>( C1( stream, 255 ) ) ) << 24U ) |
            ( static_cast<uint32_t>( static_cast<uint8_t>( C2( stream, 255 ) ) ) << 16U ) |
            ( static_cast<uint32_t>( static_cast<uint8_t>( C3( stream, 255 ) ) ) << 8U ) |
            static_cast<uint32_t>( static_cast<uint8_t>( C4( stream, 255 ) ) );
  ++output;
}

template<typename T, Extractor<T> C1, Extractor<T> C2, Extractor<T> C3, Extractor<T> C4>
void setPixelPreMultiplied( const uint8_t *stream, uint32_t *&output )
{
  uint32_t alpha = C1( stream, 255 );
  *output =
      ( static_cast<uint32_t>( static_cast<uint8_t>( alpha ) ) << 24U ) |
      ( static_cast<uint32_t>( static_cast<uint8_t>( alpha * C2( stream, 255 ) / 255 ) ) << 16U ) |
      ( static_cast<uint32_t>( static_cast<uint8_t>( alpha * C3( stream, 255 ) / 255 ) ) << 8U ) |
      static_cast<uint32_t>( static_cast<uint8_t>( alpha * C4( stream, 255 ) / 255 ) );
  ++output;
}

template<typename T, Extractor<T> R, Extractor<T> G, Extractor<T> B>
void setPixelGrayscale( const uint8_t *stream, uint8_t *&output )
{
  // Using weighted method instead of average for perceptually better results
  float value = 0.299f * R( stream, 255 ) + 0.587f * G( stream, 255 ) + 0.114f * B( stream, 255 );
  *output = static_cast<uint8_t>( value );
  ++output;
}

template<typename T, Extractor<T> R, Extractor<T> G, Extractor<T> B>
void setPixelGrayscale( const uint8_t *stream, uint16_t *&output )
{
  typedef typename std::conditional<std::is_same<T, uint8_t>::value, uint8_t, uint16_t>::type MaxType;
  constexpr MaxType max = std::numeric_limits<MaxType>::max();
  constexpr uint16_t mult = ( std::numeric_limits<uint16_t>::max() + 1 ) / ( max + 1 );
  // Using weighted method instead of average for perceptually better results
  float value = 0.299f * R( stream, max ) + 0.587f * G( stream, max ) + 0.114f * B( stream, max );
  *output = static_cast<uint16_t>( value * mult );
  ++output;
}

template<typename T, void func( const uint8_t *, T *& ), int BYTES_PER_PIXEL>
void iterateImage( const sensor_msgs::msg::Image &img, T *data )
{
  const uint8_t *stream = img.data.data();
  int offset = 0;
  for ( unsigned int row = 0; row < img.height; ++row ) {
    int base = offset;
    for ( unsigned int col = 0; col < img.width; ++col ) {
      func( stream + base, data );
      base += BYTES_PER_PIXEL;
    }
    offset += img.step;
  }
}

template<typename T, Extractor<T> R, Extractor<T> G, Extractor<T> B, Extractor<T> A, int BYTES_PER_PIXEL>
bool convertToFormat( const sensor_msgs::msg::Image &img, QVideoFrame::PixelFormat format,
                      uint8_t **data, int &num_bytes, int &bytes_per_line )
{
  // Reserve array
  switch ( format ) {
  case QVideoFrame::Format_RGB24:
  case QVideoFrame::Format_BGR24:
    num_bytes = img.width * img.height * 3;
    bytes_per_line = img.width * 3;
    break;
  case QVideoFrame::Format_RGB32:
  case QVideoFrame::Format_ARGB32:
  case QVideoFrame::Format_ARGB32_Premultiplied:
  case QVideoFrame::Format_BGR32:
  case QVideoFrame::Format_BGRA32:
  case QVideoFrame::Format_BGRA32_Premultiplied:
    num_bytes = img.width * img.height * 4;
    bytes_per_line = img.width * 4;
    break;
  case QVideoFrame::Format_Y8:
    num_bytes = img.width * img.height;
    bytes_per_line = img.width;
    break;
  case QVideoFrame::Format_Y16:
    num_bytes = img.width * img.height * 2;
    bytes_per_line = img.width * 2;
    break;
  default:
    qWarning( "Tried to convert to unknown format. This should not be happen! Please open an issue "
              "on GitHub." );
    return false;
  }

  *data = new uint8_t[num_bytes];
  switch ( format ) {
  case QVideoFrame::Format_RGB24:
    iterateImage<uint8_t, setPixel<T, R, G, B>, BYTES_PER_PIXEL>( img, *data );
    return true;
  case QVideoFrame::Format_BGR24:
    iterateImage<uint8_t, setPixel<T, B, G, R>, BYTES_PER_PIXEL>( img, *data );
    return true;
  case QVideoFrame::Format_RGB32:
    iterateImage<uint32_t, setPixel<T, returnMax<T>, R, G, B>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
  case QVideoFrame::Format_ARGB32:
    iterateImage<uint32_t, setPixel<T, A, R, G, B>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
  case QVideoFrame::Format_ARGB32_Premultiplied:
    iterateImage<uint32_t, setPixelPreMultiplied<T, A, R, G, B>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
  case QVideoFrame::Format_BGR32:
    iterateImage<uint32_t, setPixel<T, returnMax<T>, B, G, R>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
    // Contrary to docs in Qt 5.9.5 this is actually 0xAABBGGRR not 0xBBGGRRAA
  case QVideoFrame::Format_BGRA32:
    iterateImage<uint32_t, setPixel<T, A, B, G, R>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
  case QVideoFrame::Format_BGRA32_Premultiplied:
    iterateImage<uint32_t, setPixelPreMultiplied<T, A, B, G, R>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint32_t *>( *data ) );
    return true;
  case QVideoFrame::Format_Y8:
    iterateImage<uint8_t, setPixelGrayscale<T, R, G, B>, BYTES_PER_PIXEL>( img, *data );
    return true;
  case QVideoFrame::Format_Y16:
    iterateImage<uint16_t, setPixelGrayscale<T, R, G, B>, BYTES_PER_PIXEL>(
        img, reinterpret_cast<uint16_t *>( *data ) );
    return true;
  default:
    qWarning( "Tried to convert to unknown format. This should not be happen! Please open an issue "
              "on GitHub." );
  }
  return false;
}

/*!
 * The preferred formats are tried in order. They should be chosen first by their ability to represent the input data,
 * e.g., the same channels, and second by their memory usage (prefer outputs that require no copy over those that do
 * and if a copy is needed prefer formats that use less memory).
 */
template<typename T, Extractor<T> R, Extractor<T> G, Extractor<T> B, Extractor<T> A, int BYTES_PER_PIXEL>
QVideoFrame::PixelFormat
convertToClosestFormat( const sensor_msgs::msg::Image &img, uint8_t **data, int &num_bytes,
                        int &bytes_per_line, const QList<QVideoFrame::PixelFormat> &native_formats,
                        const QList<QVideoFrame::PixelFormat> &supported_formats,
                        const QList<QVideoFrame::PixelFormat> &preferred_formats )
{
  for ( QVideoFrame::PixelFormat format : preferred_formats ) {
    if ( supported_formats.contains( format ) ) {
      if ( native_formats.contains( format ) )
        return format; // No conversion necessary
      if ( convertToFormat<T, R, G, B, A, BYTES_PER_PIXEL>( img, format, data, num_bytes,
                                                            bytes_per_line ) )
        return format;
    }
  }
  return QVideoFrame::Format_Invalid;
}

QVideoFrame::PixelFormat convertFrame( const sensor_msgs::msg::Image &img, uint8_t **data,
                                       int &num_bytes, int &bytes_per_line,
                                       const QList<QVideoFrame::PixelFormat> &supported_formats )
{
  num_bytes = img.data.size();
  bytes_per_line = img.step;
  bool same_endianness = ( endian::native == endian::big ) == static_cast<bool>( img.is_bigendian );
  if ( img.encoding == sensor_msgs::image_encodings::RGB8 ) {
    return convertToClosestFormat<uint8_t, extractChannel<uint8_t, 0>, extractChannel<uint8_t, 1>,
                                  extractChannel<uint8_t, 2>, returnMax<uint8_t>, 3>(
        img, data, num_bytes, bytes_per_line, { QVideoFrame::Format_RGB24 }, supported_formats,
        { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
          QVideoFrame::Format_BGR32, QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
          QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::BGR8 ) {
    return convertToClosestFormat<uint8_t, extractChannel<uint8_t, 2>, extractChannel<uint8_t, 1>,
                                  extractChannel<uint8_t, 0>, returnMax<uint8_t>, 3>(
        img, data, num_bytes, bytes_per_line, { QVideoFrame::Format_BGR24 }, supported_formats,
        { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_RGB32, QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
          QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
          QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::MONO8 ||
              img.encoding == sensor_msgs::image_encodings::TYPE_8UC1 ) {
    return convertToClosestFormat<uint8_t, extractChannel<uint8_t, 0>, extractChannel<uint8_t, 0>,
                                  extractChannel<uint8_t, 0>, returnMax<uint8_t>, 1>(
        img, data, num_bytes, bytes_per_line, { QVideoFrame::Format_Y8 }, supported_formats,
        { QVideoFrame::Format_Y8, QVideoFrame::Format_Y16, QVideoFrame::Format_RGB24,
          QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied } );
  } else if ( img.encoding == sensor_msgs::image_encodings::MONO16 ||
              img.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ) {
    if ( same_endianness )
      return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0>, extractChannel<uint16_t, 0>,
                                    extractChannel<uint16_t, 0>, returnMax<uint16_t>, 2>(
          img, data, num_bytes, bytes_per_line, { QVideoFrame::Format_Y16 }, supported_formats,
          { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8, QVideoFrame::Format_RGB24,
            QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
            QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
            QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied } );

    return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0, true>,
                                  extractChannel<uint16_t, 0, true>,
                                  extractChannel<uint16_t, 0, true>, returnMax<uint16_t>, 2>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8, QVideoFrame::Format_RGB24,
          QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied } );
  } else if ( img.encoding == sensor_msgs::image_encodings::RGB16 ) {
    if ( same_endianness )
      return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0>, extractChannel<uint16_t, 1>,
                                    extractChannel<uint16_t, 2>, returnMax<uint16_t>, 6>(
          img, data, num_bytes, bytes_per_line, {}, supported_formats,
          { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
            QVideoFrame::Format_BGR32, QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
            QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
            QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0, true>,
                                  extractChannel<uint16_t, 1, true>,
                                  extractChannel<uint16_t, 2, true>, returnMax<uint16_t>, 6>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
          QVideoFrame::Format_BGR32, QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
          QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::BGR16 ) {
    if ( same_endianness )
      return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 2>, extractChannel<uint16_t, 1>,
                                    extractChannel<uint16_t, 0>, returnMax, 6>(
          img, data, num_bytes, bytes_per_line, {}, supported_formats,
          { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
            QVideoFrame::Format_RGB32, QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
            QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
            QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 2, true>,
                                  extractChannel<uint16_t, 1, true>,
                                  extractChannel<uint16_t, 0, true>, returnMax, 6>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_RGB32, QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
          QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
          QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::RGBA8 ) {
    // Native format is BGRA32 if little endian because then RGBA in memory is ABGR as an uint32 which is somehow
    // the actual layout of BGRA32 in Qt 5.9.5 (contrary to what the documentation says)
    QList<QVideoFrame::PixelFormat> native_formats;
    if ( endian::native == endian::little )
      native_formats = { QVideoFrame::Format_BGRA32, QVideoFrame::Format_BGR32 };
    return convertToClosestFormat<uint8_t, extractChannel<uint8_t, 0>, extractChannel<uint8_t, 1>,
                                  extractChannel<uint8_t, 2>, extractChannel<uint8_t, 3>, 4>(
        img, data, num_bytes, bytes_per_line, native_formats, supported_formats,
        { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
          QVideoFrame::Format_BGR32, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24,
          QVideoFrame::Format_RGB32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::BGRA8 ) {
    // Native format is ARGB32 if little endian because then BGRA in memory is ARGB as an uint32
    QList<QVideoFrame::PixelFormat> native_formats;
    if ( endian::native == endian::little )
      native_formats = { QVideoFrame::Format_ARGB32, QVideoFrame::Format_RGB32 };
    return convertToClosestFormat<uint8_t, extractChannel<uint8_t, 2>, extractChannel<uint8_t, 1>,
                                  extractChannel<uint8_t, 0>, extractChannel<uint8_t, 3>, 4>(
        img, data, num_bytes, bytes_per_line, native_formats, supported_formats,
        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
          QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24,
          QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::RGBA16 ) {
    if ( same_endianness )
      return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0>, extractChannel<uint16_t, 1>,
                                    extractChannel<uint16_t, 2>, extractChannel<uint16_t, 3>, 8>(
          img, data, num_bytes, bytes_per_line, {}, supported_formats,
          { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
            QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
            QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
            QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 0, true>,
                                  extractChannel<uint16_t, 1, true>, extractChannel<uint16_t, 2, true>,
                                  extractChannel<uint16_t, 3, true>, 8>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied,
          QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32,
          QVideoFrame::Format_BGR32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::BGRA16 ) {
    if ( same_endianness )
      return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 2>, extractChannel<uint16_t, 1>,
                                    extractChannel<uint16_t, 0>, extractChannel<uint16_t, 3>, 8>(
          img, data, num_bytes, bytes_per_line, {}, supported_formats,
          { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
            QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
            QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
            QVideoFrame::Format_RGB32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
    return convertToClosestFormat<uint16_t, extractChannel<uint16_t, 2, true>,
                                  extractChannel<uint16_t, 1, true>, extractChannel<uint16_t, 0, true>,
                                  extractChannel<uint16_t, 3, true>, 8>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_BGRA32, QVideoFrame::Format_ARGB32,
          QVideoFrame::Format_BGRA32_Premultiplied, QVideoFrame::Format_ARGB32_Premultiplied,
          QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB24, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_RGB32, QVideoFrame::Format_Y8, QVideoFrame::Format_Y16 } );
  } else if ( img.encoding == sensor_msgs::image_encodings::TYPE_32FC1 ) {
    if ( img.is_bigendian )
      return convertToClosestFormat<float, extractFloatChannelFromBigEndian<0>,
                                    extractFloatChannelFromBigEndian<0>,
                                    extractFloatChannelFromBigEndian<0>, returnMax<float>, 4>(
          img, data, num_bytes, bytes_per_line, {}, supported_formats,
          { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8, QVideoFrame::Format_RGB24,
            QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
            QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
            QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied } );
    return convertToClosestFormat<float, extractFloatChannelFromLittleEndian<0>,
                                  extractFloatChannelFromLittleEndian<0>,
                                  extractFloatChannelFromLittleEndian<0>, returnMax<float>, 4>(
        img, data, num_bytes, bytes_per_line, {}, supported_formats,
        { QVideoFrame::Format_Y16, QVideoFrame::Format_Y8, QVideoFrame::Format_RGB24,
          QVideoFrame::Format_BGR24, QVideoFrame::Format_RGB32, QVideoFrame::Format_BGR32,
          QVideoFrame::Format_ARGB32, QVideoFrame::Format_BGRA32,
          QVideoFrame::Format_ARGB32_Premultiplied, QVideoFrame::Format_BGRA32_Premultiplied } );
  }
  return QVideoFrame::Format_Invalid;
}
} // namespace

ImageBuffer::ImageBuffer( sensor_msgs::msg::Image::ConstSharedPtr img,
                          const QList<QVideoFrame::PixelFormat> &supported_formats )
    : QAbstractVideoBuffer( QAbstractVideoBuffer::NoHandle ), image_( std::move( img ) ),
      data_( nullptr )
{
  format_ = convertFrame( *image_, &data_, num_bytes_, bytes_per_line_, supported_formats );
}

ImageBuffer::~ImageBuffer() { delete[] data_; }

QAbstractVideoBuffer::MapMode ImageBuffer::mapMode() const { return ReadOnly; }

uchar *ImageBuffer::map( QAbstractVideoBuffer::MapMode, int *num_bytes, int *bytes_per_line )
{
  if ( num_bytes != nullptr )
    *num_bytes = num_bytes_;
  if ( bytes_per_line != nullptr )
    *bytes_per_line = bytes_per_line_;
  if ( data_ != nullptr )
    return data_;
  return const_cast<uchar *>( image_->data.data() );
}

void ImageBuffer::unmap() { }

QVideoFrame::PixelFormat ImageBuffer::format() const { return format_; }
} // namespace qml_ros2_plugin
