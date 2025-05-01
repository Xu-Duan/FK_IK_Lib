function [q_history, distance_to_goal, difference_axis, distance_to_wall] = opti_ik_space_visualize(M, S, p_goal, q_init, qL, qU, n_wall, p_wall)
    % opti_ik_space_visualize.m
    % This function performs inverse kinematics using the space Jacobian.
    % It calculates the joint angles that achieve a desired end-effector pose.

    % Inputs:
    %   M: 4x4 transformation matrix of the end-effector
    %   S: 6xN matrix of normalized twists
    %   p_des: 3x1 desired position of the end-effector
    %   q_init: 1xN initial guess for the joint angles
    %   qL: Nx1 lower joint limits
    %   qU: Nx1 upper joint limits

    assert(isequal(size(S, 1), 6), 'S must be a 6xn matrix');
    n = size(S, 2);
    assert(isequal(size(q_init), [n 1]), 'q_init must be nx1');
    assert(isequal(size(M), [4 4]), 'M must be 4x4');
    assert(isequal(size(p_goal), [3 1]), 'p_des must be 3x1');
    assert(isequal(size(qL), [n 1]), 'qL must be nx1');
    assert(isequal(size(qU), [n 1]), 'qU must be nx1');
    epsilon = 0.003;
    q_history = [q_init];
    q = q_init;
    distance_to_wall = [];
    difference_axis = [];
    distance_to_goal = [];
    error = 1;
    z_s = [0; 0; 1];
    % Get initial end-effector axis orientation
    Tsb = FK_space(M, S, q);
    R_init = Tsb(1:3, 1:3) * z_s;

    while error > epsilon
        % Forward kinematics: Calculate current end-effector configuration
        Tsb = FK_space(M, S, q);
        t = Tsb(1:3, 4);
        error = norm(t - p_goal);
        distance_to_goal = [distance_to_goal, error];
        Rz = Tsb(1:3, 1:3) * z_s;
        difference_axis = [difference_axis, norm(Rz - R_init)];
        J = J_space(S, q);
        J_alpha = J(1:3, :);
        J_p = J(4:6, :);
        distance_to_wall = [distance_to_wall, (t - p_wall)' * n_wall];
        A = J_p - vecToso3(t) * J_alpha;
        b = p_goal - t;
        % Bounds
        lb = qL - q;
        ub = qU - q;

        options = optimoptions('lsqlin','Display','none');
        delta_q = lsqlin(A, b, [], [], [], [], lb, ub, [], options);
        q = q + delta_q;
        q_history = [q_history, q];
    end
end