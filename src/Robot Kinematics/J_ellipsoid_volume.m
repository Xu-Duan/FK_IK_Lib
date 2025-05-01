function v = J_ellipsoid_volume(J)
% J_ELLIPSOID_VOLUME Calculate the volume of the manipulability ellipsoid
%
% Input:
%   J - 6xn Jacobian matrix
%
% Output:
%   v - Volume of the manipulability ellipsoid
%       Proportional to the product of singular values
%       A larger value indicates better overall manipulability
%
% Note:
%   The manipulability ellipsoid volume is a measure of the overall
%   manipulation ability in all directions. A larger volume indicates
%   better overall manipulability, but does not guarantee uniformity
%   of manipulation ability in different directions.

% Input validation
assert(size(J,1) == 6, 'Jacobian must have 6 rows');

% Calculate singular values of the Jacobian
s = svd(J);

% Calculate volume (proportional to product of singular values)
% For numerical stability, use sum of logs instead of direct product
v = prod(s);

end 