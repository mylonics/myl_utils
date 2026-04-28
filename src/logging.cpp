
#ifdef CONFIG_MYL_UTILS_LOG
#include <myl_utils/zephyr/log.h>
/* Register inside the myl_utils namespace so C++ headers that call
 * DECLARE_MYL_UTILS_LOG() inside namespace myl_utils {} can resolve
 * the log_const_myl_utils symbol without a namespace mismatch. */
namespace myl_utils {
INITIALIZE_MYL_UTILS_LOG();
} // namespace myl_utils
#endif