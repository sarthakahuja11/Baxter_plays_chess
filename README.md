# Baxter_plays_chess
Enable Baxter robot to play chess

Terminal 1:

echo "source ~/rf_ws/devel/setup.bash" >> ~/.bashrc
roslaunch baxter_gazebo baxter_world.launch

Terminal 2:

rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py

Terminal 3:

roslaunch baxter_moveit_config baxter_grippers.launch

Terminal 4:

rosrun chess_baxter spawn_chessboard_by_baxter.py OR rosrun chess_baxter spawn_chessboard_directly.py

Terminal 5:

rosrun chess_baxter pick_and_place_moveit.py

Terminal 6:

rosrun chess_baxter gazebo2tfframe.py

Delete chessboard:
rosrun chess_baxter delete_chessgame.py


Video 1: Baxter plays 5 moves

Video 2: Baxter sets up chess board (some pieces dropped)

Video 3: Baxter sets up chess board with unique pieces