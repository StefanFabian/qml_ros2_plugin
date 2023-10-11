// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TF_TRANSFORM_LISTENER_HPP
#define QML_ROS2_PLUGIN_TF_TRANSFORM_LISTENER_HPP

#include "qml_ros2_plugin/qobject_ros2.hpp"
#include "qml_ros2_plugin/time.hpp"

#include <QDateTime>
#include <memory>
#include <tf2_ros/buffer.h>

namespace qml_ros2_plugin
{

class TfTransformListener final : public QObject
{
  Q_OBJECT
private:
  TfTransformListener();

public:
  static TfTransformListener &getInstance();

  TfTransformListener( const TfTransformListener & ) = delete;

  void operator=( const TfTransformListener & ) = delete;

  ~TfTransformListener() final;

  //! Returns true if the TfTransformListener was already initialized, false otherwise.
  //! This does not mean it will already receive tf2 data as Ros2 may not be initialized yet.
  bool isInitialized() const;

  /*!
   * Checks if a transform is possible. Returns true if possible, otherwise either false or if available a message why
   * the transform failed.
   * @param target_frame The frame into which to transform.
   * @param source_frame The frame from which to transform.
   * @param time The time at which to transform in seconds.
   * @param timeout How long to block before failing in milliseconds. Set to 0 for no timeout.
   * @return True if the transform is possible, otherwise an error message (string) if available, false if not.
   */
  QVariant canTransform( const QString &target_frame, const QString &source_frame,
                         const rclcpp::Time &time = rclcpp::Time( 0 ), double timeout = 0 ) const;

  /*!
   * Checks if a transform is possible. Returns true if possible, otherwise either false or if available a message why
   * the transform failed.
   * @param target_frame The frame into which to transform.
   * @param target_time The time into which to transform.
   * @param source_frame The frame from which to transform.
   * @param source_time The time from which to transform.
   * @param fixed_frame The frame in which to treat the transform as constant in time.
   * @param timeout How long to block before failing in milliseconds. Set to 0 for no timeout.
   * @return True if the transform is possible, otherwise an error message (string) if available, false if not.
   */
  QVariant canTransform( const QString &target_frame, const rclcpp::Time &target_time,
                         const QString &source_frame, const rclcpp::Time &source_time,
                         const QString &fixed_frame, double timeout = 0 ) const;

  /*!
   * Get the transform between two frames by frame id.
   * @param target_frame The frame to which the data should be transformed.
   * @param source_frame The frame where the data originated.
   * @param time The time at which the value of the transform is desired. Set to 0 for latest.
   * @param timeout How long to block before failing in milliseconds. Set to 0 for no timeout.
   * @return A map containing a boolean valid field. If valid is true it also contains the transform.
   *   If valid is false, it might contain more information, e.g., an exception field with the name of the exception
   *    and a message field containing more information about the reason of failure.
   */
  QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                               const rclcpp::Time &time = rclcpp::Time( 0 ), double timeout = 0 );

  /*!
   * Get the transform between two frames by frame id.
   * @param target_frame The frame to which the data should be transformed.
   * @param target_time The time to which the data should be transformed. Set to 0 for latest.
   * @param source_frame The frame where the data originated.
   * @param source_time The time at which the source_frame should be evaluated. Set to 0 for latest.
   * @param fixed_frame The frame in which to assume the transform is constant in time.
   * @param timeout How long to block before failing in milliseconds. Set to 0 for no timeout.
   * @return A map containing a boolean valid field. If valid is true it also contains the transform.
   *   If valid is false, it might contain more information, e.g., an exception field with the name of the exception
   *    and a message field containing more information about the reason of failure.
   */
  QVariantMap lookUpTransform( const QString &target_frame, const rclcpp::Time &target_time,
                               const QString &source_frame, const rclcpp::Time &source_time,
                               const QString &fixed_frame, double timeout = 0 );

  tf2_ros::Buffer *buffer();

  void registerWrapper();

  //! If the count of wrappers gets to zero, the resources of this singleton will be freed.
  void unregisterWrapper();

protected:
  bool initialize();

  struct State;
  mutable std::unique_ptr<State> state_;
  std::atomic<int> wrapper_count_;
};

/*!
 * A wrapper around the TfTransformListener singleton to allow the QML engine to create and destroy it.
 */
class TfTransformListenerWrapper : public QObject
{
  Q_OBJECT
public:
  TfTransformListenerWrapper();

  ~TfTransformListenerWrapper() override;

  Q_INVOKABLE void initialize();

  //! @copydoc TfTransformListener::canTransform(const QString &, const QString &, const Time &, double)
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QString &source_frame,
                                     const QDateTime &time, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::canTransform(const QString &, const QString &, const Time &, double)
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QString &source_frame,
                                     const Time &time = Time( rclcpp::Time( 0 ) ),
                                     double timeout = 0 ) const;

  //! @copydoc TfTransformListener::canTransform(const QString &, const Time &, const QString &, const Time &, const QString &, double)
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QDateTime &target_time,
                                     const QString &source_frame, const QDateTime &source_time,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::canTransform(const QString &, const Time &, const QString &, const Time &, const QString &, double)
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const Time &target_time,
                                     const QString &source_frame, const Time &source_time,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const QString &, const Time &, double)
  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                                           const QDateTime &time, double timeout = 0 );

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const QString &, const Time &, double)
  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                                           const Time &time = Time( rclcpp::Time( 0 ) ),
                                           double timeout = 0 );

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const Time &, const QString &, const Time &, const QString &, double)
  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QDateTime &target_time,
                                           const QString &source_frame, const QDateTime &source_time,
                                           const QString &fixed_frame, double timeout = 0 );

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const Time &, const QString &, const Time &, const QString &, double)
  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const Time &target_time,
                                           const QString &source_frame, const Time &source_time,
                                           const QString &fixed_frame, double timeout = 0 );
};
} // namespace qml_ros2_plugin

#endif // QML_ROS2_PLUGIN_TF_TRANSFORM_LISTENER_HPP
