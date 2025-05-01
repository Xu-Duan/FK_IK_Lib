function [J] = J_space(S, theta)
% J_SPACE Calculate the space Jacobian matrix for a serial robot
%
% Inputs:
%   S     - 6xn matrix of normalized twists, where each column is a 6x1 twist
%          vector [omega; v] representing the joint axis in space frame
%   theta - nx1 vector of joint angles/positions
%
% Output:
%   J     - 6xn space Jacobian matrix
%
% Note:
%   - Each twist in S must be normalized (norm(omega) ≈ 1)
%   - The twists are assumed to be in the space frame
%   - The Jacobian is computed by propagating the twists through the chain

% Input validation
if nargin < 2
    error('Not enough input arguments. Need twist matrix S and joint angles theta.');
end

assert(isequal(size(S, 1), 6), 'S must be a 6xn matrix.');
n = size(S, 2);
assert(isequal(size(theta), [n 1]), 'theta must be a nx1 matrix.');
assert(n > 0, 'Input matrices cannot be empty.');

% Initialize Jacobian and cumulative transformation
J = zeros(6, n);
T_cumulative = eye(4);

% Compute Jacobian column by column
for i = 1:n
    % Check if twist is normalized
    omega = S(1:3, i);
    assert((norm(omega) - 1) < 1e-3, sprintf('Twist %d is not normalized', i));
    
    % Current column of Jacobian is the adjoint of cumulative transform
    % applied to the current twist
    J(:, i) = adjointSE3(T_cumulative) * S(:, i);
    
    % Update cumulative transformation for next iteration
    T_cumulative = T_cumulative * twistToSE3(S(:, i), theta(i));
end

end