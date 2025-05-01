function [adjS] = adjointTwist(S)
% S is a twist 6x1
assert(isequal(size(S), [6, 1]), 'S must be a 6x1 matrix.');
adjS = [vecToso3(S(1:3)) S(4:6); 0 0 0 0];
end
