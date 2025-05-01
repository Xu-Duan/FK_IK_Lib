function [R, p] = correspondence_eigen(A, B)
% Input: A, B are Nx3 matrices containing N corresponding 3D points
% Output: R is the 3x3 rotation matrix that best aligns A with B
%         p is the translation vector
assert(size(A,1) == size(B,1), 'A and B must have the same number of points');
assert(size(A,2) == 3, 'A must be a 3xN matrix');
assert(size(B,2) == 3, 'B must be a 3xN matrix');
A = A';
B = B';
% Step 3.1: Compute matrix H and its components
N = size(A, 2);
A_avg = mean(A, 2);
B_avg = mean(B, 2);

H = zeros(3, 3);
for i = 1:N
    H = H + (A(:,i) - A_avg) * (B(:,i) - B_avg)';
end

% Step 3.2: Compute G matrix
traceH = trace(H);
Delta = [H(2,3)-H(3,2); H(3,1)-H(1,3); H(1,2)-H(2,1)];
G = [traceH, Delta';
     Delta, H + H' - traceH*eye(3)];

% Step 3.3: Compute eigenvalue decomposition of G
[Q, Lambda] = eig(G);

% Step 3.4: Find the eigenvector corresponding to largest eigenvalue
[~, maxIdx] = max(diag(Lambda));
q = Q(:, maxIdx);  % This is our unit quaternion

% Convert quaternion to rotation matrix
R = quaternionToSO3(q);

% Compute translation
p = B_avg - R * A_avg;
end