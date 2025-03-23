function theta = J_inverse_kinematics(M, S, T_desired, theta_init, epsilon)
% J_inverse_kinematics Solve inverse kinematics using iterative Newton-Raphson method
%
% Inputs:
%   B          - 6xn matrix of normalized twists in body frame
%   M          - 4x4 home configuration matrix
%   T_desired  - 4x4 desired end-effector configuration matrix
%   theta_init - nx1 initial guess for joint angles
%   epsilon    - Error tolerance (default: 1e-4)
%
% Outputs:
%   theta   - nx1 vector of joint angles that achieve T_desired
%   success - Boolean indicating whether algorithm converged
%
% Note:
%   Uses Newton-Raphson iteration with space Jacobian.

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

theta = theta_init;
error = ones(n, 1);
while norm(error) > epsilon
    % Forward kinematics: Calculate current end-effector configuration
    Tsb = FK_space(M, S, theta);
    error = adjointSE3(Tsb) * SE3ToTwist(Tsb\T_desired);
    delta_theta = pinv(J_space(S, theta)) * error;
    % Update joint angles
    theta = theta + delta_theta;
end
end