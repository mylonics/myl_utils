#pragma once

#include <zephyr/kernel.h>

#include "myl_utils/noncopyable.h"

#if !defined(BUFFER_MEM_REGION)
#define BUFFER_MEM_REGION EMPTY
#endif
#define ALIGNMENT_VAR 32

/** @brief This macro helps call a function during static initialization. 
 */

#define CALL_FUNCTION_ON_STATIC_INIT(function)                   \
  class function##_class : NonCopyable<function##_class> \
  {                                                      \
  public:                                                \
    function##_class() { function(); }                   \
  }; \
  function##_class function##_object;


