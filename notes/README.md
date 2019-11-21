# Notes

## Hardware Exercise 2: pure pursuit

1. `cd /home/rey/github/udem-fall19-public`
2. `docker-compose up`
3. go to notebook, open terminal
4. in notebook terminal do `catkin build --workspace catkin_ws and source catkin_ws/devel/setup.bash`
5. in notebook terminal do `./launch_car_interface.sh`
6. in notebook terminal do `roslaunch duckietown_demos lane_following.launch` OR `roslaunch rey_pure_pursuit rey_pure_pursuit.launch`
7. open `http://localhost:6901/vnc.html` in browser (pass: `quackquack`)
8. open terminal in GUI and run `rosparam set /default/line_detector_node/verbose true`
9. run `rviz` in GUI terminal (OPTIONAL)
10. run `rqt_image_view` in gui terminal
11. run `rosrun rey_pure_pursuit pure_pursuit.py`
12. Run keyboard control in sim: `dts duckiebot keyboard_control default --network udemfall19public_duckietown-docker-net --sim --cli --base_image duckietown/dt-core:daffy`

## Software Exercise 6: computer vision

1. `cd /home/rey/github/udem-fall19-public` then `docker-compose up`
2. open new terminal in docker
3. `./launch_car_interface.sh`
4. `roslaunch sw_06_line_detector lane_following.launch`
5. go to `http://localhost:6901/vnc.html` (pass is `quackquack`)
6. in vnc, open terminal, and run `rosparam set /default/line_detector_node/verbose true`
7. in vnc, run `rviz` (OPTIONAL)
8. in rviz, click add then do by topic, pick `/duckiebot_visualizer/segment_list_markers/` and `/lane_pose_visualizer_node/lane_pose_markers`
9. Run keyboard control in sim: `dts duckiebot keyboard_control default --network udemfall19public_duckietown-docker-net  --sim --base_image duckietown/dt-core:daffy`
10. Run keyboard control in real: `dts duckiebot keyboard_control bebek --base_image duckietown/dt-core:daffy-amd64`
11. Pushing image to docker and pull it from the robot (see below)
12. to run on real robot `dts duckiebot demo --demo_name XXXX --package_name XXXX --duckiebot_name bebek --image rrwiyatn/XXXX:XXXX`

## Software Exercise 7: particle filter

1. `cd /home/rey/github/udem-fall19-public` then `docker-compose up`
2. open new terminal in docker
3. `./launch_car_interface.sh`
4. `roslaunch sw_07_lane_filter lane_following.launch`
5. go to `http://localhost:6901/vnc.html` (pass is `quackquack`)
6. in vnc, open terminal, and run `rosparam set /default/line_detector_node/verbose true`
7. in vnc, run `rviz` (OPTIONAL)
8. in rviz, click add then do by topic, pick `/duckiebot_visualizer/segment_list_markers/` and `/lane_pose_visualizer_node/lane_pose_markers`
9. Run keyboard control in sim: `dts duckiebot keyboard_control default --network udemfall19public_duckietown-docker-net  --sim --base_image duckietown/dt-core:daffy`
    - Note: to check network name, do `docker network ls`
10. Run keyboard control in real: `dts duckiebot keyboard_control bebek --base_image duckietown/dt-core:daffy-amd64`
11. Pushing image to docker and pull it from the robot (see below)
12. to run on real robot `dts duckiebot demo --demo_name lane_following --package_name sw_07_lane_filter --duckiebot_name bebek --image rrwiyatn/sw07-se:master-arm32v7`
    - NOTE: `dts duckiebot demo --demo_name <your-launch-file-name> --package_name <your-package-name> --duckiebot_name <your-duckiebot-name> --image duckietown/<repo-name>:<branch-name>`

## Pushing image to docker and pull it from the robot

1. `cd` to our package dir
2. make sure we have pushed all the changes made in our package to github
3. push to docker `dts devel build --push -u rrwiyatn` (CONFIRM)
4. pull new image from robot `docker -H bebek.local pull <yourimage>` (e.g., `docker -H bebek.local pull rrwiyatn/rey_core:v1-arm32v7`)

## For logging

1. `ssh duckie@bebek.local`
2. `docker exec -it rrwiyatn/rey_core:v1-arm32v7 /bin/bash`
3. `docker -H bebek.local cp <container_id>:/code/catkin_ws/src/rey_core/packages/rey_pure_pursuit/dump.txt /home/rey/`

## Pulling from upstream master

1. `cd /home/rey/github/udem-fall19-public`
2. `git pull upstream master`
3. `git submodule init`
4. `git submodule update`
5. `git submodule foreach "(git checkout daffy; git pull)"`
6. `cd simulation`
7. `git checkout 4693556`

## Other things to remember

1. sometimes we need to `docker-compose build`
2. to build catkin_ws in docker, open terminal in docker then `catkin build --workspace catkin_ws`, then `source catkin_ws/devel/setup.bash`
