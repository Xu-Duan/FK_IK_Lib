function theta = redundancy_resolution(M, S, T_desired, theta_init, epsilon)
% REDUNDANCY_RESOLUTION Solve inverse kinematics with redundancy resolution
% using manipulability measure as secondary objective
%
% Inputs:
%   M          - 4x4 home configuration matrix
%   S          - 6xn matrix of normalized twists in space frame
%   T_desired  - 4x4 desired end-effector configuration matrix
%   theta_init - nx1 initial guess for joint angles
%   epsilon    - Error tolerance (default: 1e-4)
%
% Outputs:
%   theta   - nx1 vector of joint angles that achieve T_desired
%   success - Boolean indicating whether algorithm converged

% Set default parameters if not provided
if nargin < 5
    epsilon = 1e-4;
end

% Input validation
assert(isequal(size(S, 1), 6), 'S must be a 6xn matrix');
n = size(S, 2);
assert(isequal(size(theta_init), [n 1]), 'theta_init must be nx1');
assert(isequal(size(M), [4 4]), 'M must be 4x4');
assert(isequal(size(T_desired), [4 4]), 'T_desired must be 4x4');

% Parameters for redundancy resolution
alpha = 0.1;  % Step size for secondary objective
theta = theta_init;
error = ones(n, 1);
while norm(error) > epsilon
    % Forward kinematics: Calculate current end-effector configuration
    Tsb = FK_space(M, S, theta);
    error = adjointSE3(Tsb) * SE3ToTwist(Tsb\T_desired);
    % Calculate space Jacobian
    J = J_space(S, theta);
    
    % Calculate manipulability measure gradient
    w = J_ellipsoid_volume(J);  % Current manipulability measure
    dw = zeros(n, 1);
    delta = 1e-4;  % Small perturbation for numerical gradient
    % Numerical gradient of manipulability measure
    for i = 1:n
        theta_perturbed = theta;
        theta_perturbed(i) = theta_perturbed(i) + delta;
        w_perturbed = J_ellipsoid_volume(J_space(S, theta_perturbed));
        dw(i) = (w_perturbed - w) / delta;
    end
    delta_theta_primary = pinv(J) * error;
    % Calculate null space projector
    N = eye(n) - pinv(J) * J;
    % Combine primary and secondary objectives
    delta_theta = delta_theta_primary + alpha * N * dw;
    % Update joint angles
    theta = theta + delta_theta;
end
end
