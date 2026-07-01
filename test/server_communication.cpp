// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"

#include <qml_ros2_plugin/action_server.hpp>
#include <qml_ros2_plugin/action_server_goal_handle.hpp>
#include <qml_ros2_plugin/qos.hpp>
#include <qml_ros2_plugin/ros2.hpp>
#include <qml_ros2_plugin/service_server.hpp>

#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <QCoreApplication>
#include <QJSEngine>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace qml_ros2_plugin;
using namespace std::chrono_literals;

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using Fibonacci = example_interfaces::action::Fibonacci;

rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
rclcpp::Node::SharedPtr node;

// The servers are held in a unique_ptr so an early ASSERT_* that returns from the test still deletes
// the server. Otherwise it would stay advertised on the shared node and contaminate later tests.

void processEvents()
{
  QCoreApplication::processEvents();
  executor->spin_some();
}

bool waitFor( const std::function<bool()> &pred, std::chrono::milliseconds timeout = 1s )
{
  auto start = std::chrono::steady_clock::now();
  while ( ( std::chrono::steady_clock::now() - start ) < timeout ) {
    processEvents();
    if ( pred() )
      return true;
    std::this_thread::sleep_for( 1ms );
  }
  return false;
}

void waitFor( std::chrono::milliseconds timeout )
{
  waitFor( []() { return false; }, timeout );
}

TEST( ServerCommunication, serviceServerSync )
{
  QJSEngine engine;
  auto server = std::make_unique<ServiceServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  server->setProcessRequest(
      engine.evaluate( "(function (req, id) { return { sum: req.a + req.b }; })" ) );
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/add_two_ints" );

  auto client = node->create_client<AddTwoInts>( "/server_communication/add_two_ints" );
  ASSERT_TRUE( waitFor( [&]() { return client->service_is_ready(); }, 5s ) );
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 5;
  request->b = 17;
  auto future = client->async_send_request( request );
  ASSERT_TRUE( waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( future.get()->sum, 22 );
}

TEST( ServerCommunication, serviceServerDeferred )
{
  QJSEngine engine;
  auto server = std::make_unique<ServiceServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  QJSValue watcher = engine.newObject();
  QJSValue process = engine
                         .evaluate( "(function (watcher) { return function (req, id) { "
                                    "watcher.id = id; watcher.sum = req.a + req.b; "
                                    "return undefined; }; })" )
                         .call( { watcher } );
  server->setProcessRequest( process );
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/deferred" );

  auto client = node->create_client<AddTwoInts>( "/server_communication/deferred" );
  ASSERT_TRUE( waitFor( [&]() { return client->service_is_ready(); }, 5s ) );
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 100;
  request->b = 23;
  auto future = client->async_send_request( request );
  // Wait until the request reached the deferred callback (id was stashed).
  ASSERT_TRUE( waitFor( [&]() { return watcher.hasProperty( "id" ); }, 3s ) );
  // Regression guard for the babel_fish patch: the response must not be auto-sent. The client
  // future must stay unresolved across several event cycles until we explicitly respond.
  waitFor( 200ms );
  ASSERT_NE( future.wait_for( 0s ), std::future_status::ready );
  const int id = watcher.property( "id" ).toInt();
  const int sum = watcher.property( "sum" ).toInt();
  server->sendResponse( id, { { "sum", sum } } );
  ASSERT_TRUE( waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( future.get()->sum, 123 );
}

TEST( ServerCommunication, serviceServerNoOpQoSPreservesDeferred )
{
  QJSEngine engine;
  auto server = std::make_unique<ServiceServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  QJSValue watcher = engine.newObject();
  QJSValue process = engine
                         .evaluate( "(function (watcher) { return function (req, id) { "
                                    "watcher.id = id; watcher.sum = req.a + req.b; "
                                    "return undefined; }; })" )
                         .call( { watcher } );
  server->setProcessRequest( process );
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/noop_qos" );

  auto client = node->create_client<AddTwoInts>( "/server_communication/noop_qos" );
  ASSERT_TRUE( waitFor( [&]() { return client->service_is_ready(); }, 5s ) );
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 40;
  request->b = 2;
  auto future = client->async_send_request( request );
  ASSERT_TRUE( waitFor( [&]() { return watcher.hasProperty( "id" ); }, 3s ) );
  // Re-write the (already default) QoS. This is a no-op and must NOT drop the in-flight deferred
  // request: without the equality guard tryCreate would clear pending_requests_ and the later
  // sendResponse would be lost, hanging the still-connected client.
  server->setQoS( QoSWrapper( rclcpp::ServicesQoS() ) );
  const int id = watcher.property( "id" ).toInt();
  const int sum = watcher.property( "sum" ).toInt();
  server->sendResponse( id, { { "sum", sum } } );
  ASSERT_TRUE( waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( future.get()->sum, 42 );
}

TEST( ServerCommunication, serviceServerNoHandler )
{
  auto server = std::make_unique<ServiceServer>();
  // No processRequest set -> the server must answer with a default response so the caller is not
  // stranded.
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/no_handler" );

  auto client = node->create_client<AddTwoInts>( "/server_communication/no_handler" );
  ASSERT_TRUE( waitFor( [&]() { return client->service_is_ready(); }, 5s ) );
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 1;
  request->b = 2;
  auto future = client->async_send_request( request );
  ASSERT_TRUE( waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( future.get()->sum, 0 ); // default-initialized response
}

TEST( ServerCommunication, serviceServerReturnsArray )
{
  QJSEngine engine;
  auto server = std::make_unique<ServiceServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  // Returning an array (instead of an object) is a user error. It must not be mistaken for a valid
  // response; the caller receives the default response instead.
  server->setProcessRequest(
      engine.evaluate( "(function (req, id) { return [req.a + req.b]; })" ) );
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/returns_array" );

  auto client = node->create_client<AddTwoInts>( "/server_communication/returns_array" );
  ASSERT_TRUE( waitFor( [&]() { return client->service_is_ready(); }, 5s ) );
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 5;
  request->b = 17;
  auto future = client->async_send_request( request );
  ASSERT_TRUE( waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( future.get()->sum, 0 ); // default response, the array was rejected
}

TEST( ServerCommunication, serviceServerRetarget )
{
  QJSEngine engine;
  auto server = std::make_unique<ServiceServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  server->setProcessRequest(
      engine.evaluate( "(function (req, id) { return { sum: req.a + req.b }; })" ) );
  server->setType( "example_interfaces/srv/AddTwoInts" );
  server->setName( "/server_communication/retarget_a" );

  auto sendAndCheck = [&]( const std::string &name, int a, int b ) {
    auto client = node->create_client<AddTwoInts>( name );
    if ( !waitFor( [&]() { return client->service_is_ready(); }, 3s ) )
      return false;
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    auto future = client->async_send_request( request );
    if ( !waitFor( [&]() { return future.wait_for( 0s ) == std::future_status::ready; }, 3s ) )
      return false;
    return future.get()->sum == a + b;
  };

  ASSERT_TRUE( sendAndCheck( "/server_communication/retarget_a", 3, 4 ) );
  // Recreate on the same name by changing the QoS to a different value (a no-op QoS write is now
  // guarded and would not recreate). Reset-before-create must avoid an rcl "service already exists"
  // error.
  server->setQoS( QoSWrapper( rclcpp::ServicesQoS() ).keep_last( 5 ) );
  ASSERT_TRUE( sendAndCheck( "/server_communication/retarget_a", 5, 6 ) );
  // Re-target to a different name at runtime.
  server->setName( "/server_communication/retarget_b" );
  ASSERT_TRUE( sendAndCheck( "/server_communication/retarget_b", 8, 9 ) );
  // The old name is no longer served.
  auto client_a = node->create_client<AddTwoInts>( "/server_communication/retarget_a" );
  EXPECT_TRUE( waitFor( [&]() { return !client_a->service_is_ready(); }, 3s ) );
}

TEST( ServerCommunication, actionServerHappyPath )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  ActionServerGoalHandle *accepted_handle = nullptr;
  QObject::connect( server.get(), &ActionServer::goalAccepted,
                    [&]( ActionServerGoalHandle *handle ) { accepted_handle = handle; } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci" );

  auto action_client =
      rclcpp_action::create_client<Fibonacci>( node, "/server_communication/fibonacci" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  std::vector<int32_t> feedback_seq;
  Fibonacci::Result::SharedPtr result_msg;
  bool got_result = false;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.feedback_callback = [&]( rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
                                   const std::shared_ptr<const Fibonacci::Feedback> fb ) {
    feedback_seq = fb->sequence;
  };
  options.result_callback = [&]( const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &r ) {
    result_msg = r.result;
    got_result = true;
  };
  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal, options );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr ) << "Goal was not accepted.";
  ASSERT_TRUE( waitFor( [&]() { return accepted_handle != nullptr; }, 3s ) );

  // Publish feedback and confirm it is delivered before finishing the goal, so the feedback is not
  // raced by the terminal result.
  accepted_handle->publishFeedback( { { "sequence", QVariantList{ 0, 1, 1 } } } );
  EXPECT_TRUE( waitFor( [&]() { return !feedback_seq.empty(); }, 3s ) );
  EXPECT_EQ( feedback_seq, ( std::vector<int32_t>{ 0, 1, 1 } ) );

  accepted_handle->succeed( { { "sequence", QVariantList{ 0, 1, 1, 2, 3 } } } );
  ASSERT_TRUE( waitFor( [&]() { return got_result; }, 3s ) );
  ASSERT_NE( result_msg, nullptr );
  EXPECT_EQ( result_msg->sequence, ( std::vector<int32_t>{ 0, 1, 1, 2, 3 } ) );
}

TEST( ServerCommunication, actionServerReject )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  server->setHandleGoal(
      engine.evaluate( "(function (goal, goalId) { return goal.order <= 3; })" ) );
  bool accepted_emitted = false;
  QObject::connect( server.get(), &ActionServer::goalAccepted,
                    [&]( ActionServerGoalHandle * ) { accepted_emitted = true; } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_reject" );

  auto action_client =
      rclcpp_action::create_client<Fibonacci>( node, "/server_communication/fibonacci_reject" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  Fibonacci::Goal goal;
  goal.order = 100; // rejected by handleGoal
  auto gh_future = action_client->async_send_goal( goal );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( gh_future.get(), nullptr ) << "Goal should have been rejected.";
  EXPECT_FALSE( accepted_emitted );
}

TEST( ServerCommunication, actionServerHandleGoalThrows )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  // A throwing handleGoal must reject the goal. A thrown Error is a truthy QJSValue, so without an
  // explicit isError() check the goal would be accepted despite the handler failing.
  server->setHandleGoal(
      engine.evaluate( "(function (goal, goalId) { throw new Error('boom'); })" ) );
  bool accepted_emitted = false;
  QObject::connect( server.get(), &ActionServer::goalAccepted,
                    [&]( ActionServerGoalHandle * ) { accepted_emitted = true; } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_throw" );

  auto action_client =
      rclcpp_action::create_client<Fibonacci>( node, "/server_communication/fibonacci_throw" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( gh_future.get(), nullptr ) << "Goal should have been rejected after handleGoal threw.";
  EXPECT_FALSE( accepted_emitted );
}

TEST( ServerCommunication, actionServerCancel )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  server->setHandleCancel( engine.evaluate( "(function (handle) { return true; })" ) );
  ActionServerGoalHandle *accepted_handle = nullptr;
  bool cancel_requested = false;
  QObject::connect( server.get(), &ActionServer::goalAccepted, [&]( ActionServerGoalHandle *handle ) {
    accepted_handle = handle;
    // Do not call canceled() synchronously inside the cancel slot; only record the request.
    QObject::connect( handle, &ActionServerGoalHandle::cancelRequested,
                      [&]() { cancel_requested = true; } );
  } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_cancel" );

  auto action_client =
      rclcpp_action::create_client<Fibonacci>( node, "/server_communication/fibonacci_cancel" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::UNKNOWN;
  bool got_result = false;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.result_callback = [&]( const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &r ) {
    result_code = r.code;
    got_result = true;
  };
  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal, options );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_TRUE( waitFor( [&]() { return accepted_handle != nullptr; }, 3s ) );

  auto cancel_future = action_client->async_cancel_goal( goal_handle );
  ASSERT_TRUE( waitFor( [&]() { return cancel_requested; }, 3s ) );
  ASSERT_TRUE(
      waitFor( [&]() { return cancel_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_TRUE( waitFor( [&]() { return accepted_handle->isCanceling(); }, 2s ) );

  // Finalize the goal from the main thread (outside the blocking cancel slot).
  accepted_handle->canceled( { { "sequence", QVariantList{ 0, 1 } } } );
  ASSERT_TRUE( waitFor( [&]() { return got_result; }, 3s ) );
  EXPECT_EQ( result_code, rclcpp_action::ResultCode::CANCELED );
}

TEST( ServerCommunication, actionServerCancelRejectedWithoutHandler )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  // No handleCancel set: cancels must be rejected so the goal is not silently abandoned.
  ActionServerGoalHandle *accepted_handle = nullptr;
  bool cancel_requested = false;
  QObject::connect( server.get(), &ActionServer::goalAccepted, [&]( ActionServerGoalHandle *handle ) {
    accepted_handle = handle;
    QObject::connect( handle, &ActionServerGoalHandle::cancelRequested,
                      [&]() { cancel_requested = true; } );
  } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_cancel_reject" );

  auto action_client = rclcpp_action::create_client<Fibonacci>(
      node, "/server_communication/fibonacci_cancel_reject" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::UNKNOWN;
  bool got_result = false;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.result_callback = [&]( const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &r ) {
    result_code = r.code;
    got_result = true;
  };
  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal, options );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_TRUE( waitFor( [&]() { return accepted_handle != nullptr; }, 3s ) );

  auto cancel_future = action_client->async_cancel_goal( goal_handle );
  ASSERT_TRUE(
      waitFor( [&]() { return cancel_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  // Server rejected the cancel: no goal is transitioning to canceling.
  EXPECT_TRUE( cancel_future.get()->goals_canceling.empty() );
  // Give an (erroneous) cancel notification a chance to propagate, then assert none happened.
  waitFor( 200ms );
  EXPECT_FALSE( cancel_requested );
  EXPECT_FALSE( accepted_handle->isCanceling() );

  // The goal is unaffected and can still complete normally.
  accepted_handle->succeed( { { "sequence", QVariantList{ 0, 1, 1, 2, 3 } } } );
  ASSERT_TRUE( waitFor( [&]() { return got_result; }, 3s ) );
  EXPECT_EQ( result_code, rclcpp_action::ResultCode::SUCCEEDED );
}

TEST( ServerCommunication, actionServerCancelSynchronousCanceled )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  server->setHandleCancel( engine.evaluate( "(function (handle) { return true; })" ) );
  ActionServerGoalHandle *accepted_handle = nullptr;
  bool cancel_requested = false;
  QObject::connect( server.get(), &ActionServer::goalAccepted, [&]( ActionServerGoalHandle *handle ) {
    accepted_handle = handle;
    // Mirror the example: finalize synchronously inside the cancelRequested handler. This must be a
    // valid CANCELING->CANCELED transition; previously cancelRequested fired while the goal was
    // still EXECUTING and canceled() failed with an invalid-transition error.
    QObject::connect( handle, &ActionServerGoalHandle::cancelRequested, [handle, &cancel_requested]() {
      cancel_requested = true;
      handle->canceled( { { "sequence", QVariantList{ 0, 1 } } } );
    } );
  } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_cancel_sync" );

  auto action_client = rclcpp_action::create_client<Fibonacci>(
      node, "/server_communication/fibonacci_cancel_sync" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::UNKNOWN;
  bool got_result = false;
  rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
  options.result_callback = [&]( const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &r ) {
    result_code = r.code;
    got_result = true;
  };
  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal, options );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_TRUE( waitFor( [&]() { return accepted_handle != nullptr; }, 3s ) );

  auto cancel_future = action_client->async_cancel_goal( goal_handle );
  ASSERT_TRUE( waitFor( [&]() { return cancel_requested; }, 3s ) );
  ASSERT_TRUE(
      waitFor( [&]() { return cancel_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  // The synchronous canceled() must have produced a CANCELED result for the client.
  ASSERT_TRUE( waitFor( [&]() { return got_result; }, 3s ) );
  EXPECT_EQ( result_code, rclcpp_action::ResultCode::CANCELED );
}

TEST( ServerCommunication, actionServerAbortsOpenGoalsOnDestruction )
{
  QJSEngine engine;
  auto server = std::make_unique<ActionServer>();
  QJSValue server_js = engine.newQObject( server.get() );
  ActionServerGoalHandle *accepted_handle = nullptr;
  QObject::connect( server.get(), &ActionServer::goalAccepted,
                    [&]( ActionServerGoalHandle *handle ) { accepted_handle = handle; } );
  server->setType( "example_interfaces/action/Fibonacci" );
  server->setName( "/server_communication/fibonacci_destroy" );

  auto action_client =
      rclcpp_action::create_client<Fibonacci>( node, "/server_communication/fibonacci_destroy" );
  ASSERT_TRUE( waitFor( [&]() { return action_client->action_server_is_ready(); }, 5s ) );
  // action_server_is_ready() can report ready before the underlying services are matched for
  // delivery; settle briefly so the first goal request is not dropped (graph discovery race).
  waitFor( 1s );

  Fibonacci::Goal goal;
  goal.order = 5;
  auto gh_future = action_client->async_send_goal( goal );
  ASSERT_TRUE(
      waitFor( [&]() { return gh_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  auto goal_handle = gh_future.get();
  ASSERT_NE( goal_handle, nullptr );
  ASSERT_TRUE( waitFor( [&]() { return accepted_handle != nullptr; }, 3s ) );

  // Request the result and let the request reach the server before destroying it, so the abort
  // issued during destruction has a pending result request to answer.
  auto result_future = action_client->async_get_result( goal_handle );
  waitFor( 500ms );

  // Destroy the server with the goal still open: it must abort the goal so the client receives a
  // result instead of waiting for the goal to expire.
  server.reset();
  ASSERT_TRUE(
      waitFor( [&]() { return result_future.wait_for( 0s ) == std::future_status::ready; }, 3s ) );
  EXPECT_EQ( result_future.get().code, rclcpp_action::ResultCode::ABORTED );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  node = rclcpp::Node::make_shared( "server_communication" );
  executor = rclcpp::executors::SingleThreadedExecutor::SharedPtr(
      new rclcpp::executors::SingleThreadedExecutor() );
  executor->add_node( node );
  Ros2QmlSingletonWrapper wrapper;
  wrapper.init( "server_communication_qml" );
  int result = RUN_ALL_TESTS();
  executor->remove_node( node );
  executor.reset();
  node.reset();
  wrapper.shutdown();
  // Match the rclcpp::init above so the (test-owned) global default context is cleaned up; the
  // plugin deliberately never tears the global context down itself.
  rclcpp::shutdown();
  return result;
}
