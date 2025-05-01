function [T] = FK_body(M, B, theta)
% M is a matrix of 4x4 of initial configuration
% B is a matrix of 6xn of twist
% theta is a vector nx1
assert(isequal(size(M), [4 4]), 'M must be a 6xn matrix.');
assert(isequal(size(B, 1), 6), 'B must be a 6xn matrix.');
n = size(B, 2);
assert(isequal(size(theta), [n 1]), 'theta must be a nx1 matrix.');
T = eye(4, 4);
for i = n:-1:1
    T = twistToSE3(B(:, i), theta(i)) * T;
end
T =  M * T;
end