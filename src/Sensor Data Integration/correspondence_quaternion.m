function [R, p] = correspondence_quaternion(A, B)
% Input: A, B are Nx3 matrices containing N corresponding 3D points
% Output: R is the 3x3 rotation matrix that best aligns A with B
%         p is the translation vector
assert(size(A,1) == size(B,1), 'A and B must have the same number of points');
assert(size(A,2) == 3, 'A must be a 3xN matrix');
assert(size(B,2) == 3, 'B must be a 3xN matrix');

A = A';
B = B';
% Compute centroids
N = size(A, 2);
A_avg = mean(A, 2);
B_avg = mean(B, 2);

% Center the point sets
A_centered = A - A_avg;
B_centered = B - B_avg;

% Initialize M matrix
M = zeros(4, 4);

for i = 1:N
    % Get centered points
    a = A_centered(:,i);
    b = B_centered(:,i);
    
    % Compute b-a and b+a
    b_minus_a = b - a;
    b_plus_a = b + a;
    
    % Build the matrix for this point pair
    Mi = [0, b_minus_a';
         b_minus_a, vecToso3(b_plus_a)];
    
    % Add to total M matrix
    M = M + Mi' * Mi;
end

% Solve for quaternion using eigenvalue decomposition
[V, D] = eig(M);
% Get eigenvector corresponding to smallest eigenvalue
[~, minIdx] = min(diag(D));
q = V(:, minIdx);

% Convert quaternion to rotation matrix
% q = [s; v] where s is scalar part and v is vector part
R = quaternionToSO3(q);

% Compute translation
p = B_avg - R * A_avg;
end