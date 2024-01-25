# Week 2
- Obstacle Avoidance Python port: turtlebot3_drive_python.py

# Week 3
- Drive Forward 4seconds, and turn for 1 second, based on counter: turtlebot3_fsm.py
- More states such as turning left and right randomly, no obstacle avoidance: turtlebot3_random_walk.py

# Week 4
- Drive forward by certain distance, turn by random angle, obstacle avoidance: robot_controller.py
- rviz markers

# Week 5
- Item detector: data=[auro_interfaes.msg.Item(x=0, diameter=34, colour='red')] (x=0 item is centered, diameter gets bigger as u get closer, and colour.)
- Using week 4 launch (empty world with walls), and run week 5 item spawner, we get world with some items. collect by driving into them. we use ```ros2 run turtlebot3_teleop teleop_keyboard``` to control the bot and get items.
- also run week 5 item detector to see the robot's camera and data mentioned above
- Instruction on how to run multi robots and have a topic subscribe to each one

# Week 6
- Runs multi robots in week 4 world.
- Talks about faults in scanning, types of faults n stuff like that
- Faults only show data 50% of the time or something like that.

# Week 7
- Launching a world, mapping it using cartographer SLAM and saving it
- 
