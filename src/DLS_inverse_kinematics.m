function theta = DLS_inverse_kinematics(M, S, T_desired, theta_init, k, epsilon)
% INVERSE_KINEMATICS Solve inverse kinematics using iterative Newton-Raphson method
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
%   Uses Newton-Raphson iteration with body Jacobian.
%   May not converge for some initial conditions or unreachable poses.

% Set default parameters if not provided
if nargin < 6
    epsilon = 1e-4;
end
if nargin < 5
    k = 0.1;
end
manipulability_epsilon = 1e-2;
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
    J = J_space(S, theta);
    if J_ellipsoid_volume(J) < manipulability_epsilon
    % Calculate joint angle update using damped least squares
        delta_theta = J' / (J * J' + k^2 * eye(6)) * error;
        delta_theta
    else
        delta_theta = pinv(J) * error;
    end
    % Update joint angles
    theta = theta + delta_theta;
end
end 