#!/usr/bin/env python

import roslaunch

package = 'muevai_tf'
executable = 'muevai_os.launch'
node = roslaunch.core.Node(package, executable)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print (process.is_alive())
process.stop()
