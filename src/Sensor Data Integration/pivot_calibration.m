function [p_tracker, p_tip, rms_error] = pivot_calibration(data, obs_per_frame, num_frames)
% PIVOT_CALIBRATION Performs pivot calibration using EM tracking data
%   [p_dimple, t_G, rms_error] = pivot_calibration(data)
%   Input:
%       data - Matrix containing the EM tracking data
%   Output:
%       p_dimple - Position of dimple in tracker coordinates
%       t_G - Probe tip coordinates in probe coordinates
%       rms_error - Root mean square error of the calibration

assert(size(data,1) == obs_per_frame * num_frames, 'data must have obs_per_frame * num_frames rows');
assert(size(data,2) == 3, 'data must have 3 columns');

% Reshape the data into frames
% Each frame will be obs_per_frame × 3
G = zeros(obs_per_frame, 3, num_frames);
for i_frame = 1:num_frames
    G(:,:,i_frame) = data(obs_per_frame*(i_frame-1)+1:obs_per_frame*i_frame, :);
end

% Step a: Define local "probe" coordinate system using first frame
% Compute centroid G0 of first frame
G0 = mean(G(:,:,1), 1);

% Translate all observations relative to G0 to get g_j
g = zeros(size(G));
for k = 1:num_frames
    for j = 1:obs_per_frame
        g(j,:,k) = G(j,:,k) - G0;
    end
end

% Step b: Compute transformations F_G for each frame
% Using correspondence_quaternion to find the optimal rigid transformation
F_G = zeros(4, 4, num_frames);

for k = 1:num_frames
    % Get current frame's points
    P = G(:,:,k);  % N×3 matrix of points in frame k
    Q = g(:,:,1);  % N×3 matrix of points in reference frame (frame 1)
    
    % Use correspondance_SVD to get the transformation
    [R, t] = correspondence_quaternion(Q, P);
    
    % Create homogeneous transformation matrix
    F_G(:,:,k) = RandpToSE3(R, t);
end

% Step c: Set up and solve the system for pivot point
% We'll use least squares to solve for t_G

% Create the system of equations
A = zeros(3*num_frames, 6);
b = zeros(3*num_frames, 1);

for k = 1:num_frames
    R_k = F_G(1:3,1:3,k);
    t_k = F_G(1:3,4,k);
    
    A(3*k-2:3*k, 1:3) = eye(3);
    A(3*k-2:3*k, 4:6) = -R_k;
    b(3*k-2:3*k) = t_k;
end

% Solve the system using least squares
x = pinv(A) * b;

% Extract results
p_tracker= x(1:3);  % Position of dimple in tracker coordinates
p_tip = x(4:6);      % Probe tip coordinates in probe coordinates

% Calculate RMS error
errors = zeros(num_frames, 1);
for k = 1:num_frames
    predicted_dimple = F_G(1:3,1:3,k) * p_tip + F_G(1:3,4,k);
    errors(k) = norm(predicted_dimple - p_tracker);
end
rms_error = sqrt(mean(errors.^2));
end 