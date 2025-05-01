clear; clc
%% Initialization
M = [sqrt(2)/2 sqrt(2)/2 0 0.088; sqrt(2)/2 -sqrt(2)/2 0 0; 0 0 -1 0.926-0.1; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; -1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]]/1000;
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
qL = deg2rad([-166; -101; -166; -176; -166; -1; -166]);
qU = deg2rad([166; 101; 166; -4; 166; 215; 166]);
q_init = zeros(7, 1);
q_init(4) = -pi/2;
q_init(6) = pi/2;
%% First case:
p_des = [0.75; 0; 0.3];
n_wall = [0; 0; 1];
p_wall = [0; 0; 0.29];

% Run comparison
metrics = compare_ik_methods(M, S, p_des, q_init, qL, qU, n_wall, p_wall, "ik_methods_comparison_1.png");

%% Second case:
p_des = [0.0; 0.5; 0.3];
n_wall = [1; 0; -1];
p_wall = [0; 0; 0.5];
metrics = compare_ik_methods(M, S, p_des, q_init, qL, qU, n_wall, p_wall, "ik_methods_comparison_2.png");

%% Third case:
p_des = [-0.4; 0.3; 0.6];
n_wall = [0.5; 0.5*sqrt(3); 0];
p_wall = [-0.35; 0.10; 0];
metrics = compare_ik_methods(M, S, p_des, q_init, qL, qU, n_wall, p_wall, "ik_methods_comparison_3.png");


%%
function [metrics] = compare_ik_methods(M, S, p_des, q_init, qL, qU, n_wall, p_wall, filename)
    % compare_ik_methods.m
    % This function compares different IK methods and plots their performance metrics
    
    % Inputs:
    %   M: 4x4 transformation matrix of the end-effector
    %   S: 6xN matrix of normalized twists
    %   p_des: 3x1 desired position of the end-effector
    %   q_init: 1xN initial guess for the joint angles
    %   qL: Nx1 lower joint limits
    %   qU: Nx1 upper joint limits
    %   n_wall: 3x1 normal vector of the virtual wall
    %   p_wall: 3x1 point on the virtual wall
    
    % Output:
    %   metrics: Structure containing the metrics for each method
    
    % Run all four methods
    [q_history_1, distance_to_goal_1, difference_axis_1, distance_to_wall_1] = opti_ik_space_visualize(M, S, p_des, q_init, qL, qU, n_wall, p_wall);
    [q_history_2, distance_to_goal_2, difference_axis_2, distance_to_wall_2] = opti_ik_angle_space_visualize(M, S, p_des, q_init, qL, qU, n_wall, p_wall);
    [q_history_3, distance_to_goal_3, difference_axis_3, distance_to_wall_3] = opti_ik_wall_visualize(M, S, p_des, q_init, qL, qU, n_wall, p_wall);
    [q_history_4, distance_to_goal_4, difference_axis_4, distance_to_wall_4] = opti_ik_angle_wall_visualize(M, S, p_des, q_init, qL, qU, n_wall, p_wall);
    
    % Store metrics in structure
    metrics.ik = struct('q_history', q_history_1, ...
                       'distance_to_goal', distance_to_goal_1, ...
                       'difference_axis', difference_axis_1, ...
                       'distance_to_wall', distance_to_wall_1);
    
    metrics.angle = struct('q_history', q_history_2, ...
                          'distance_to_goal', distance_to_goal_2, ...
                          'difference_axis', difference_axis_2, ...
                          'distance_to_wall', distance_to_wall_2);
    
    metrics.wall = struct('q_history', q_history_3, ...
                         'distance_to_goal', distance_to_goal_3, ...
                         'difference_axis', difference_axis_3, ...
                         'distance_to_wall', distance_to_wall_3);
    
    metrics.angle_wall = struct('q_history', q_history_4, ...
                               'distance_to_goal', distance_to_goal_4, ...
                               'difference_axis', difference_axis_4, ...
                               'distance_to_wall', distance_to_wall_4);
    
    % Plot metrics comparison
    figure('Position', [100, 100, 1200, 800]);
    
    % Distance to goal
    subplot(3,1,1);
    plot(distance_to_goal_1, 'LineWidth', 2); hold on;
    plot(distance_to_goal_2, 'LineWidth', 2);
    plot(distance_to_goal_3, 'LineWidth', 2);
    plot(distance_to_goal_4, 'LineWidth', 2);
    title('Distance to Goal Over Iterations');
    xlabel('Iteration');
    ylabel('Distance (m)');
    legend('ik', 'angle', 'wall', 'angle\_wall');
    grid on;
    
    % Difference axis angle
    subplot(3,1,2);
    plot(vecnorm(difference_axis_1, 2, 1), 'LineWidth', 2); hold on;
    plot(vecnorm(difference_axis_2, 2, 1), 'LineWidth', 2);
    plot(vecnorm(difference_axis_3, 2, 1), 'LineWidth', 2);
    plot(vecnorm(difference_axis_4, 2, 1), 'LineWidth', 2);
    title('Orientation Error Over Iterations');
    xlabel('Iteration');
    ylabel('Error (rad)');
    legend('ik', 'angle', 'wall', 'angle\_wall');
    grid on;
    
    % Distance to wall
    subplot(3,1,3);
    plot(distance_to_wall_1, 'LineWidth', 2); hold on;
    plot(distance_to_wall_2, 'LineWidth', 2);
    plot(distance_to_wall_3, 'LineWidth', 2);
    plot(distance_to_wall_4, 'LineWidth', 2);
    title('Distance to Wall Over Iterations');
    xlabel('Iteration');
    ylabel('Distance (m)');
    legend('ik', 'angle', 'wall', 'angle\_wall');
    grid on;
    
    % Save the figure
    saveas(gcf, filename);
end

