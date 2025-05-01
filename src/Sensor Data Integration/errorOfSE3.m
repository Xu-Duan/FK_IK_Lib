function [translation_error, rotation_error] = errorOfSE3(X1,X2)
% Input: 
%   X - 4xN transformation matrix SO3
% Output: 
%   translation_error and rotation_error

% Comparition
p1 = X1(1:3, 4);
p2 = X2(1:3, 4);
translation_error = norm(p1 - p2);  % scalar 

R1 = X1(1:3, 1:3);
R2 = X2(1:3, 1:3);
R_err = R1' * R2;
theta_err_rad = acos( (trace(R_err) - 1) / 2 );  % in radians
rotation_error = rad2deg(theta_err_rad);          % in degrees