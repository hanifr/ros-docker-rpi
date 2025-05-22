#!/bin/bash

echo "🎮 Robot Teleop Control 🎮"
echo "=========================="
echo "Use keyboard to control the robot:"
echo "  w - forward"
echo "  s - backward"
echo "  a - turn left"
echo "  d - turn right"
echo "  q - quit"
echo "  space - stop"
echo ""
echo "Press any key to start..."
read -n 1

# Ensure we clean up on exit
trap "echo 'Stopping robot'; docker-compose exec ros bash -c 'source /opt/ros/noetic/setup.bash && rostopic pub -1 /cmd_vel geometry_msgs/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"'" EXIT

# Variable to track current speed
LINEAR_SPEED=0.5
ANGULAR_SPEED=1.0

while true; do
    echo -n "Command > "
    read -n 1 cmd
    echo ""
    
    case $cmd in
        w)
            echo "Moving forward (speed: $LINEAR_SPEED)"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: $LINEAR_SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        s)
            echo "Moving backward (speed: $LINEAR_SPEED)"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: -$LINEAR_SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        a)
            echo "Turning left (speed: $ANGULAR_SPEED)"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_SPEED}}'"
            ;;
        d)
            echo "Turning right (speed: $ANGULAR_SPEED)"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -$ANGULAR_SPEED}}'"
            ;;
        " ")
            echo "Stopping robot"
            docker-compose exec ros bash -c "source /opt/ros/noetic/setup.bash && \
                rostopic pub -1 /cmd_vel geometry_msgs/Twist \
                '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"
            ;;
        +)
            LINEAR_SPEED=$(echo "$LINEAR_SPEED + 0.1" | bc)
            ANGULAR_SPEED=$(echo "$ANGULAR_SPEED + 0.1" | bc)
            echo "Speed increased - Linear: $LINEAR_SPEED, Angular: $ANGULAR_SPEED"
            ;;
        -)
            if (( $(echo "$LINEAR_SPEED > 0.1" | bc -l) )); then
                LINEAR_SPEED=$(echo "$LINEAR_SPEED - 0.1" | bc)
                ANGULAR_SPEED=$(echo "$ANGULAR_SPEED - 0.1" | bc)
                echo "Speed decreased - Linear: $LINEAR_SPEED, Angular: $ANGULAR_SPEED"
            else
                echo "Speed already at minimum"
            fi
            ;;
        q)
            echo "Quitting"
            exit 0
            ;;
        *)
            echo "Unknown command: '$cmd'"
            ;;
    esac
done