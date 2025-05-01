N = 10;
A = randn(N, 3);

R = [0 1 0;
    1 0 0;
    0 0 -1];

p = [1; 2; 3];
for i = 1:N
    B(i, :) = R * A(i, :)' + p;
end

%%
[R, p] = correspondance_SVD(A, B)

%%
[R, p] = correspondence_eigen(A, B)

%%
[R, p] = correspondence_quaternion(A, B)