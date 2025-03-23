%%
M = [0 1 0 0.088; 1 0 0 0; 0 0 -1 0.926; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; -1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]]/1000;
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];

%% Case 1
theta_0 = [0.195 1.9 0.7 -0.7 0.50 1.0 0.6]';
T_desired = FK_space(M, S, theta_0)
theta_init = [0; pi/2; 0; -pi/2; 0; 0; 0];
theta_ik = redundancy_resolution(M, S, T_desired, theta_init);
T_get = FK_space(M, S, theta_ik)

%% Case 2
theta_0 = [0.01 0.02 0.03 -0.1 0.05 0.06 0.07]';
T_desired = FK_space(M, S, theta_0)
theta_init = [0; 1; 0; -0.5; 0; 0; 0];
theta_ik = redundancy_resolution(M, S, T_desired, theta_init);
T_get = FK_space(M, S, theta_ik)