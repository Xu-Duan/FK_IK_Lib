%% Initialization
M = [0 1 0 0.088; 1 0 0 0; 0 0 -1 0.926; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; -1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]]/1000;
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
B = adjointSE3(inv(M)) * S;
%% Case 1: Benchmark
theta = [0 0 0 0 0 0 0]';
FK_body(M, B, theta)
FK_space(M, S, theta)

%% Case 2
theta = [0 deg2rad(-40) 0 deg2rad(-110) 0 deg2rad(90) 0]';
FK_body(M, B, theta)
FK_space(M, S, theta)

%% Case 3
theta = [deg2rad(20) deg2rad(-40) deg2rad(30) deg2rad(-110) deg2rad(-25) deg2rad(90) deg2rad(10)]';
FK_body(M, B, theta)
FK_space(M, S, theta)