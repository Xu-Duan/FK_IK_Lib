%% Initialization
M = [0 1 0 0.088; 1 0 0 0; 0 0 -1 0.926; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; -1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]]/1000;
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
robot = loadrobot("frankaEmikaPanda");
homeConfig = homeConfiguration(robot);
B = adjointSE3(inv(M)) * S;

%% Case 1: Benchmark 
theta = [0 0 0 0 0 0 0]';
T = FK_space(M, S, theta);
R = T(1:3, 1:3);
J_b = J_body(B, theta)
adjointSE3(T) * J_b
J_s = J_space(S, theta)

J_trans = [R zeros(3); zeros(3) R] * J_b
theta = [0 0 0 0 0 0 -pi/4]';
for i = 1:7
    homeConfig(i).JointPosition = theta(i);
end
geometricJacobian(robot, homeConfig, 'panda_hand')

%% Case 2
theta = [0 deg2rad(-40) 0 deg2rad(-110) 0 deg2rad(90) 0]';
T = FK_space(M, S, theta);
R = T(1:3, 1:3);
J = J_body(B, theta);
J_trans = [R zeros(3); zeros(3) R] * J
theta = [0 deg2rad(-40) 0 deg2rad(-110) 0 deg2rad(90) -pi/4]';
for i = 1:7
    homeConfig(i).JointPosition = theta(i);
end
geometricJacobian(robot, homeConfig, 'panda_hand')

%% Case 3
theta = [deg2rad(20) deg2rad(-40) deg2rad(30) deg2rad(-110) deg2rad(-25) deg2rad(90) deg2rad(10)]';
T = FK_space(M, S, theta);
R = T(1:3, 1:3);
[omega, theta2] = SO3ToAxisAngle(R);
r = omega * theta2;
A = eye(3) - (1-cos(norm(r)))/ norm(r)^2 * vecToso3(r) + (norm(r)-sin(norm(r)))/ norm(r)^3 * (vecToso3(r))^2;
J = J_body(B, theta);
J_trans = [R zeros(3); zeros(3) R] * J
theta = [deg2rad(20) deg2rad(-40) deg2rad(30) deg2rad(-110) deg2rad(-25) deg2rad(90) deg2rad(-35)]';
for i = 1:7
    homeConfig(i).JointPosition = theta(i);
end
geometricJacobian(robot, homeConfig, 'panda_hand')

%% Case 4: Validation
theta = [0 deg2rad(-40) 0 deg2rad(-110) 0 deg2rad(90) 0]';
delta = pi/200 * ones(7, 1);
theta_delta = theta + delta;
adjointSE3(FK_space(M, S, theta_delta)) * SE3ToTwist(FK_space(M, S, theta) \ FK_space(M, S, theta_delta))
J_space(S, theta) * delta