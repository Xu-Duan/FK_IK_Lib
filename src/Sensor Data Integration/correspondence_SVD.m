function [R, p] = correspondence_SVD(A, B)
% Input: A, B are Nx3 matrices containing N corresponding 3D points
% Output: R is the 3x3 rotation matrix that best aligns A with B
%         p is the translation vector
assert(size(A,1) == size(B,1), 'A and B must have the same number of points');
assert(size(A,2) == 3, 'A must be a 3xN matrix');
assert(size(B,2) == 3, 'B must be a 3xN matrix');

A = A';
B = B';
% Step 3.1: Compute matrix H
H = zeros(3,3);
N = size(A,2);  % Number of points
A_avg = mean(A,2);
B_avg = mean(B,2);

for i = 1:N
    H = H + (A(:,i) - A_avg) * (B(:,i) - B_avg)';
end

% Step 3.2: Compute SVD of H
[U, S, V] = svd(H);

% Step 3.3: Compute R = VU^T
R = V * U';

% Step 3.4: Verify det(R) = 1
if abs(det(R) - 1) > 1e-10
    warning('Algorithm may fail: det(R) ≠ 1');
    % Fix for reflections if necessary
    V(:,3) = V(:,3) * -1;
    R = V * U';
end

p = B_avg - R * A_avg;

end