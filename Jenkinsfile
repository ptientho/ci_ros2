pipeline {
      agent any 
      stages {
          stage('Print + list current directory') {
              steps {
                  sh 'pwd'
                  sh 'ls -al'
              }
          }
          stage('Change directory to Dockerfile') {
              steps {
                  sh '''
                  cd /home/user/ros2_ws/src/ros2_ci
                  ls -la
                  '''
              }
          }
          stage('Build image') {
              steps {
                  sh '''
                  if [ ! -f "Dockerfile" ]; then
                    echo "Error: Dockerfile not found."
                    exit 1
                  else
                    sudo docker build -t ci:ros2 .
                  fi
                  sudo docker images
                  '''
              }
          }
          stage('Test waypoints server') {
              steps {
                  sh '
                  sudo docker run --rm -v /tmp/.X11-unix/:/tmp/.X11-unix/ ci:ros2 /bin/bash -c "ros2 run tortoisebot_waypoints tortoisebot_waypoints_action_server_node; colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+"'
                    
              }
          }
          
      }
  }