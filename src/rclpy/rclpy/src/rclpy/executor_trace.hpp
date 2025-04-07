#ifndef RCLPY__EXECUTOR_TRACE_HPP_
#define RCLPY__EXECUTOR_TRACE_HPP_


#include <pybind11/pybind11.h>


namespace py = pybind11;

namespace rclpy
{
  void trace_callback_start(uint64_t callback_id);

  void trace_callback_end(uint64_t callback_id);

  void trace_executor_execute(uint64_t callback_id);

  void trace_executor_get_next_ready();

  void trace_executor_wait_for_work(int64_t);
}

#endif