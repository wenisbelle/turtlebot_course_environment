#include "executor_trace.hpp"

#include <tracetools/tracetools.h>

namespace rclpy
{
  void trace_callback_start(const uint64_t callback_id) {
    TRACEPOINT(callback_start,
      reinterpret_cast<const void *>(callback_id),
      false
    );
  }

  void trace_callback_end(const uint64_t callback_id) {
    TRACEPOINT(callback_end,
      reinterpret_cast<const void *>(callback_id)
    );
  }

  void trace_executor_execute(uint64_t callback_id) {
    TRACEPOINT(rclcpp_executor_execute,
      reinterpret_cast<const void *>(callback_id)
    );
  }

  void trace_executor_get_next_ready() {
    TRACEPOINT(rclcpp_executor_get_next_ready);
  }

  void trace_executor_wait_for_work(int64_t timeout) {
    TRACEPOINT(rclcpp_executor_wait_for_work,
      timeout
    );
  }
}