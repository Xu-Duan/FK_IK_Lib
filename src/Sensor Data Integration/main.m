%% section 1
clear; clc
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]= data_quaternion();
[X] = hand_eye_calibration(q_Robot_config', q_camera_config', t_Robot_config', t_camera_config')

%% section 2
clear; clc
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ] = data_quaternion_noisy();
%% subsection a
[X] = hand_eye_calibration(q_Robot_config', q_camera_config', t_Robot_config', t_camera_config')

%% subsection b
rng(2025)
X = randperm(10, 5);
q_Robot_config_half = q_Robot_config(X, :);
q_camera_config_half = q_camera_config(X, :);
t_Robot_config_half = t_Robot_config(X, :);
t_camera_config_half = t_camera_config(X, :);
[X] = hand_eye_calibration(q_Robot_config_half', q_camera_config_half', t_Robot_config_half', t_camera_config_half')
