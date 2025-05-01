function [T] = twistToSE3(S, theta)
% TWISTTOSE3 Convert a twist to an SE(3) transformation matrix using matrix exponential
%
% Inputs:
%   S     - 6x1 twist vector [omega; v] where:
%          omega is the normalized angular velocity (3x1)
%          v is the linear velocity (3x1)
%   theta - Scalar parameter (typically time or distance)
%
% Output:
%   T     - 4x4 homogeneous transformation matrix in SE(3)
%
% Note:
%   The input twist S must be normalized (norm(omega) ≈ 1)

assert(isequal(size(S), [6, 1]), 'S must be a 6x1 matrix.');
assert(isequal(size(theta), [1, 1]), 'theta must be a 1x1 matrix.');

omega = S(1:3);
assert((norm(omega) - 1) < 1e-3, 'twist is not normalized')
v = S(4:6);
omega_hat = vecToso3(omega);
I = eye(3);

T = zeros(4, 4);

T(1:3, 1:3) = axisAngleToSO3(omega, theta);
T(1:3, 4) = (I* theta + (1-cos(theta))*omega_hat + (theta - sin(theta))*omega_hat^2)*v;
T(4, 4) = 1;
end