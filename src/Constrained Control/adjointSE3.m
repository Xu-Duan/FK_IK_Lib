function [adjT] = adjointSE3(T)
assert(isequal(size(T), [4, 4]), 'S must be a 4x4 matrix.');
R = T(1:3, 1:3);
p = T(1:3, 4);
adjT = [R zeros(3, 3); vecToso3(p)*R R];
end