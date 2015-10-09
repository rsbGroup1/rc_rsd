FILE(REMOVE_RECURSE
  "CMakeFiles/rc_grasp_generate_messages_cpp"
  "devel/include/rc_grasp/Move.h"
  "devel/include/rc_grasp/setConfiguration.h"
  "devel/include/rc_grasp/Open.h"
  "devel/include/rc_grasp/getConfiguration.h"
  "devel/include/rc_grasp/grabBrick.h"
  "devel/include/rc_grasp/Stop.h"
  "devel/include/rc_grasp/getIsMoving.h"
  "devel/include/rc_grasp/stopRobot.h"
  "devel/include/rc_grasp/getQueueSize.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rc_grasp_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
