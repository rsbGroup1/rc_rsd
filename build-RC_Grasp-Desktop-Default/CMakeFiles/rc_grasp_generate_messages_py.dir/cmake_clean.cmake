FILE(REMOVE_RECURSE
  "CMakeFiles/rc_grasp_generate_messages_py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_Move.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_setConfiguration.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_Open.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_getConfiguration.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_grabBrick.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_Stop.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_getIsMoving.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_stopRobot.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/_getQueueSize.py"
  "devel/lib/python2.7/dist-packages/rc_grasp/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rc_grasp_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
