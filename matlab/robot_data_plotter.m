function robot_data = read_robot_csv(filename)
    % Read CSV file exported from web interface
    if nargin < 1
        [file, path] = uigetfile('*.csv', 'Select Robot Data CSV');
        filename = fullfile(path, file);
    end
    
    % Read the CSV data
    data = readtable(filename);
    
    % Extract data into structured format
    robot_data.timestamp = data.timestamp;
    robot_data.x_position = data.x_position;
    robot_data.y_position = data.y_position;
    robot_data.velocity = data.velocity;
    robot_data.time_elapsed = data.time_elapsed;
    
    % Create visualizations
    figure('Name', 'Robot Data Analysis', 'Position', [100, 100, 1200, 800]);
    
    % Plot 1: Robot trajectory
    subplot(2, 2, 1);
    plot(robot_data.x_position, robot_data.y_position, 'b-', 'LineWidth', 2);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Robot Trajectory');
    grid on;
    axis equal;
    
    % Plot 2: Position vs time
    subplot(2, 2, 2);
    plot(robot_data.time_elapsed, robot_data.x_position, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(robot_data.time_elapsed, robot_data.y_position, 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Position (m)');
    title('Position vs Time');
    legend('X Position', 'Y Position');
    grid on;
    
    % Plot 3: Velocity profile
    subplot(2, 2, 3);
    plot(robot_data.time_elapsed, robot_data.velocity, 'g-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Velocity Profile');
    grid on;
    
    % Plot 4: Statistics
    subplot(2, 2, 4);
    stats = {
        sprintf('Total Distance: %.2f m', sum(sqrt(diff(robot_data.x_position).^2 + diff(robot_data.y_position).^2)));
        sprintf('Max Velocity: %.2f m/s', max(robot_data.velocity));
        sprintf('Avg Velocity: %.2f m/s', mean(robot_data.velocity));
        sprintf('Total Time: %.1f s', max(robot_data.time_elapsed));
        sprintf('Data Points: %d', length(robot_data.time_elapsed))
    };
    
    text(0.1, 0.8, stats, 'FontSize', 12, 'VerticalAlignment', 'top');
    xlim([0 1]); ylim([0 1]);
    title('Statistics');
    axis off;
    
    % Calculate performance metrics
    robot_data.total_distance = sum(sqrt(diff(robot_data.x_position).^2 + diff(robot_data.y_position).^2));
    robot_data.max_velocity = max(robot_data.velocity);
    robot_data.avg_velocity = mean(robot_data.velocity);
    robot_data.total_time = max(robot_data.time_elapsed);
    
    fprintf('Robot Data Analysis Complete\n');
    fprintf('Total Distance: %.2f m\n', robot_data.total_distance);
    fprintf('Maximum Velocity: %.2f m/s\n', robot_data.max_velocity);
    fprintf('Average Velocity: %.2f m/s\n', robot_data.avg_velocity);
    fprintf('Total Time: %.1f s\n', robot_data.total_time);
end

% Function to connect to robot in real-time (requires MATLAB ROS Toolbox)
function setup_matlab_ros_connection()
    % This requires MATLAB ROS Toolbox
    try
        % Connect to ROS 2 network
        ros2 connect;
        
        % Create subscriber for robot position
        odom_sub = ros2subscriber('/odom', 'nav_msgs/Odometry', @odom_callback);
        
        % Create publisher for robot commands
        cmd_pub = ros2publisher('/cmd_vel', 'geometry_msgs/Twist');
        
        fprintf('MATLAB connected to ROS 2 robot\n');
        
        % Example: Send movement command from MATLAB
        cmd_msg = ros2message(cmd_pub);
        cmd_msg.linear.x = 0.5;  % Move forward
        send(cmd_pub, cmd_msg);
        
    catch ME
        fprintf('ROS 2 connection failed: %s\n', ME.message);
        fprintf('Make sure MATLAB ROS Toolbox is installed\n');
    end
end

function odom_callback(msg)
    % Callback function for position updates
    persistent fig_handle;
    
    if isempty(fig_handle) || ~isvalid(fig_handle)
        fig_handle = figure('Name', 'Real-time Robot Position');
    end
    
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    
    figure(fig_handle);
    plot(x, y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
    hold on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title(sprintf('Robot Position: (%.2f, %.2f)', x, y));
    grid on;
    drawnow;
end
