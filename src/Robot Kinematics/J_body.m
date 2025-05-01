function [J] = J_body(B, theta)
% J_BODY Calculate the body Jacobian matrix for a serial robot
%
% Inputs:
%   B     - 6xn matrix of normalized twists, where each column is a 6x1 twist
%          vector [omega; v] representing the joint axis in body frame
%   theta - nx1 vector of joint angles/positions
%
% Output:
%   J     - 6xn body Jacobian matrix
%
% Note:
%   - Each twist in S must be normalized (norm(omega) ≈ 1)
%   - The twists are assumed to be in the body frame
%   - The Jacobian is computed by propagating the twists backwards through the chain

% Input validation
if nargin < 2
    error('Not enough input arguments. Need twist matrix S and joint angles theta.');
end

assert(isequal(size(B, 1), 6), 'S must be a 6xn matrix.');
n = size(B, 2);
assert(isequal(size(theta), [n 1]), 'theta must be a nx1 matrix.');
assert(n > 0, 'Input matrices cannot be empty.');

% Initialize Jacobian and cumulative transformation
J = zeros(6, n);
T_cumulative = eye(4);

% Compute Jacobian column by column, starting from the last joint
for i = n:-1:1
    % Check if twist is normalized
    omega = B(1:3, i);
    assert((norm(omega) - 1) < 1e-3, sprintf('Twist %d is not normalized', i));
    
    % Current column of Jacobian is the current twist
    J(:, i) = adjointSE3(T_cumulative) * B(:, i);
    
    % Update cumulative transformation for next iteration
    % Note: We multiply on the right since we're working in body frame
    T_cumulative = T_cumulative * twistToSE3(B(:, i), -theta(i));
end

end