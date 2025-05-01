clear; clc
%% Initialization
M = [sqrt(2)/2 sqrt(2)/2 0 0.088; sqrt(2)/2 -sqrt(2)/2 0 0; 0 0 -1 0.926-0.1; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; -1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]]/1000;
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
qL = deg2rad([-166; -101; -166; -176; -166; -1; -166]);
qU = deg2rad([166; 101; 166; -4; 166; 215; 166]);
q_init = zeros(7, 1);
q_init(4) = -pi/2;
q_init(6) = pi/2;
robot = loadrobot("frankaEmikaPanda");
homeConfig = homeConfiguration(robot);
for i_joint = 1:7
    homeConfig(i_joint).JointPosition = q_init(i_joint);
end
figure(1); axis([-1 1 -1 1 0 1])
%% First case:
p_des = [0.75; 0; 0.3];
show(robot,homeConfig,PreservePlot=false); hold on;
axis([-1 1 -1 1 0 1])
plotTransforms(p_des',eul2quat([0 0 0]),FrameSize=0.2);
saveas(gcf, "ik_angle_space_test_1_0.png");
case1 = homeConfiguration(robot);
q_history = opti_ik_angle_space(M, S, p_des, q_init, qL, qU);
for i = 1:size(q_history, 2)
    for i_joint = 1:7
        case1(i_joint).JointPosition = q_history(i_joint, i);
    end
    show(robot, case1,PreservePlot=false);
    saveas(gcf, sprintf("ik_angle_space_test_1_%d.png", i));
end
T = FK_space(M, S, q_history(:, end));
fprintf("Final position: %f %f %f\n", T(1:3, 4));
fprintf("Final direction: %f %f %f\n", T(1:3, 1:3) * [0; 0; 1]);
%% Second case:
p_des = [0.0; 0.5; 0.3];
show(robot,homeConfig,PreservePlot=false); hold on;
axis([-1 1 -1 1 0 1])
plotTransforms(p_des',eul2quat([0 0 0]),FrameSize=0.2);
saveas(gcf, "ik_angle_space_test_2_0.png");
case2 = homeConfiguration(robot);
q_history = opti_ik_angle_space(M, S, p_des, q_init, qL, qU);
for i = 1:size(q_history, 2)
    for i_joint = 1:7
        case2(i_joint).JointPosition = q_history(i_joint, i);
    end
    show(robot, case2,PreservePlot=false);
    saveas(gcf, sprintf("ik_angle_space_test_2_%d.png", i));
end
T = FK_space(M, S, q_history(:, end));
fprintf("Final position: %f %f %f\n", T(1:3, 4));
fprintf("Final direction: %f %f %f\n", T(1:3, 1:3) * [0; 0; 1]);
%% Third case:
p_des = [-0.4; 0.3; 0.6];
show(robot,homeConfig,PreservePlot=false); hold on;
axis([-1 1 -1 1 0 1])
plotTransforms(p_des',eul2quat([0 0 0]),FrameSize=0.2);
saveas(gcf, "ik_angle_space_test_3_0.png");
case3 = homeConfiguration(robot);
q_history = opti_ik_angle_space(M, S, p_des, q_init, qL, qU);
for i = 1:size(q_history, 2)
    for i_joint = 1:7
        case3(i_joint).JointPosition = q_history(i_joint, i);
    end
    show(robot, case3, PreservePlot=false);
    saveas(gcf, sprintf("ik_angle_space_test_3_%d.png", i));
end
T = FK_space(M, S, q_history(:, end));
fprintf("Final position: %f %f %f\n", T(1:3, 4));
fprintf("Final direction: %f %f %f\n", T(1:3, 1:3) * [0; 0; 1]);
