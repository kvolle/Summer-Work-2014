if [false]; then
rosws set hector_slam --git https://github.com/tu-darmstadt-ros-pkg/hector_slam
rosws set hector_quadrotor --git https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
rosws set hector_localization --git https://github.com/tu-darmstadt-ros-pkg/hector_localization
rosws set hector_gazebo --git https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
rosws set hector_models --git https://github.com/tu-darmstadt-ros-pkg/hector_models
rosws set hector_navigation --git https://github.com/tu-darmstadt-ros-pkg/hector_navigation
rosws set hector_quadrotor_apps --git https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor_apps
rosws set hector_visualization --git https://github.com/tu-darmstadt-ros-pkg/hector_visualization
rosws set hector_common --git https://github.com/tu-darmstadt-ros-pkg/hector_common

rosws update
fi

rosmake hector_common
rosmake hector_slam
rosmake	hector_quadrotor
rosmake hector_localization
rosmake hector_gazebo
rosmake hector_models
rosmake hector_navigation
rosmake hector_quadrotor_apps
rosmake hector_visualization
