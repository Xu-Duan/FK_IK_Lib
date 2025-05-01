function [T] = FK_space(M, S, theta)
% FK_space calculates the configuration of the end-effector
% Inputs:
%     M is a matrix of 4x4 of initial configuration of end-effector
%     S is a matrix of 6xn of screw axes
%     theta is a joint angle vector nx1
% Outputs:
%   T   - 4x4 the configuration of the end-effector

assert(isequal(size(M), [4 4]), 'M must be a 6xn matrix.');
assert(isequal(size(S, 1), 6), 'S must be a 6xn matrix.');
n = size(S, 2);
assert(isequal(size(theta), [n 1]), 'theta must be a nx1 matrix.');
T = eye(4, 4);
for i = 1:n
    T = T * twistToSE3(S(:, i), theta(i));
end
T = T * M;
end