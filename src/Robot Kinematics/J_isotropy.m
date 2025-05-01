function mu1 = J_isotropy(J)
% J_ISOTROPY Calculate the isotropy index of a Jacobian matrix
%
% Input:
%   J  - 6xn Jacobian matrix
%
% Output:
%   mu1 - Isotropy index (ratio of min to max singular values)
%         Range is [1,inf] where 1 indicates perfect isotropy
%
% Note:
%   The isotropy index indicates how uniform the manipulator's ability
%   to move is in different directions. A value closer to 1 indicates
%   more isotropic behavior.

% Input validation
assert(size(J,1) == 6, 'Jacobian must have 6 rows');

% Calculate singular values of the Jacobian
s = svd(J);

% Extract minimum and maximum singular values
s_min = min(s);
s_max = max(s);

% Calculate isotropy index (ratio of min to max singular value)
% If s_max is zero, return 0 to avoid division by zero
if s_min < eps
    mu1 = inf;
else
    mu1 = s_max / s_min;
end

end