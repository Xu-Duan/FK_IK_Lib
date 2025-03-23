%%
M = [0 1 0 88; 1 0 0 0; 0 0 -1 926; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]];
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
%%
theta = [0.2 pi/2 0.78 -pi/6 0.56 pi/4 0.34]';
J_space(S, theta)
%%
theta_delta = theta + [0; 0; 0; 0; 0.001; 0; 0];
SE3ToTwist(FK_space(M, S, theta_delta)) - SE3ToTwist(FK_space(M, S, theta))
res = J_space(S, theta) * [0; 0; 0; 0; 0.001; 0; 0]

%%
theta_delta = theta + [0; 0; 0; 0; 0.01; 0; 0];
deltaT = FK_space(M, S, theta_delta) / FK_space(M, S, theta);
[twist, delta_theta] = SE3ToTwist(deltaT);
twist * delta_theta / 0.01