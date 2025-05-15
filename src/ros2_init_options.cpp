// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros2_plugin/ros2_init_options.hpp"

namespace qml_ros2_plugin
{

Ros2InitOptions::Ros2InitOptions( QObject *parent ) : QObject( parent ) { }

Ros2InitOptions::~Ros2InitOptions() = default;

QObject *Ros2InitOptions::setNamespace( const QString &ns )
{
  namespace_ = ns;
  return this;
}

QString Ros2InitOptions::getNamespace() const { return namespace_; }

QObject *Ros2InitOptions::setDomainId( int domain_id )
{
  options_.set_domain_id( domain_id );
  return this;
}

int Ros2InitOptions::getDomainId() { return options_.get_domain_id(); }

QObject *Ros2InitOptions::useDefaultDomainId()
{
  options_.use_default_domain_id();
  return this;
}

const rclcpp::InitOptions &Ros2InitOptions::rclcppInitOptions() const { return options_; }
} // namespace qml_ros2_plugin
