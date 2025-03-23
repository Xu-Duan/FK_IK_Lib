function theta = J_transpose_kinematics(M, S, T_desired, theta_init, epsilon)
% J_TRANSPOSE_KINEMATICS Solve inverse kinematics using Jacobian Transpose method
%
% Inputs:
%   M          - 4x4 home configuration matrix
%   S          - 6xn matrix of screw axes in space frame
%   T_desired  - 4x4 desired end-effector configuration matrix
%   theta_init - nx1 initial guess for joint angles
%   epsilon    - Error tolerance (default: 1e-4)
%
% Outputs:
%   theta   - nx1 vector of joint angles that achieve T_desired
%   success - Boolean indicating whether algorithm converged
%
% Note:
%   Uses Jacobian Transpose method which is simpler and more stable than
%   Newton-Raphson, though it may converge more slowly.
%   The update rule is: theta = theta + alpha * J' * error

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
    J = J_space(S, theta);
    % Update joint angles
    alpha = error' * (J * J') * error / norm(J * J' * error)^2;
    theta = theta + alpha * J' * error;
end
end