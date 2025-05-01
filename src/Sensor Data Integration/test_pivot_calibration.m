clear; clc
b_tip = [1; 2; 3];
N = 100;
g = randn(N , 3);
R = [1 0 0;
    0 1 0;
    0 0 1];
p = [1; 2; 3];

H = [];
for i = 1:N
    H = [H; (R * g(i, :)' + p)'];
end
% [R, p] = correspondence_SVD(g, G)
b_post = R * b_tip + p
% [post, tip, err] = pivot_calibration2(H, N, 1)
%% set 2
R = [0 1 0;
    1 0 0;
    0 0 -1];

p = b_post - R * b_tip;

for i = 1:N
    H = [H; (R * g(i, :)' + p)'];
end
[post, tip, err] = pivot_calibration2(H(1:N, :), N, 1)
[post, tip, err] = pivot_calibration2(H, N, 2)