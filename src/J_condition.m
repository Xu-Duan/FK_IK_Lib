function mu2 = J_condition(J)
% J_CONDITION Calculate the condition number of a Jacobian matrix
%
% Input:
%   J   - 6xn Jacobian matrix
%
% Output:
%   mu2 - Condition number (norm(J) * norm(pinv(J)))
%         Range is [1,inf] where 1 indicates perfect conditioning
%
% Note:
%   The condition number indicates numerical stability and sensitivity
%   to errors. A value closer to 1 indicates better conditioning.
%   This implementation uses the 2-norm (spectral norm).

% Input validation
assert(size(J,1) == 6, 'Jacobian must have 6 rows');

% Calculate condition number using 2-norm
% For 2-norm, condition number is the ratio of largest to smallest singular value
A = J * J';
s = svd(A);
s_min = min(s);
s_max = max(s);

if s_min < eps
    mu2 = inf;
else
    mu2 = s_max / s_min;
end

end 