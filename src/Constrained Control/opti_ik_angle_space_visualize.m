function [q_history, distance_to_goal, difference_axis, distance_to_wall] = opti_ik_angle_space_visualize(M, S, p_goal, q_init, qL, qU, n_wall, p_wall)
    % opti_ik_angle_space_visualize.m
    % This function performs inverse kinematics using the space Jacobian.
    % It calculates the joint angles that achieve a desired end-effector pose.
    % while minimizing the change in the direction of the end-effector axis

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
    epsilon1 = 0.003;
    epsilon2 = 0.01;
    z_s = [0; 0; 1];
    eta = 0.1;
    q_history = [q_init];
    q = q_init;
    distance_to_goal = [];
    distance_to_wall = [];
    difference_axis = [];
    error1 = 1;
    error2 = 1;
    Tsb = FK_space(M, S, q);
    R_init = Tsb(1:3, 1:3) * z_s;
    max_iter = 20;
    beta = 0.5;
    while error1 > epsilon1 || error2 > epsilon2
        % Forward kinematics: Calculate current end-effector configuration
        Tsb = FK_space(M, S, q);
        Rz = Tsb(1:3, 1:3) * z_s;
        t = Tsb(1:3, 4);
        error1 = norm(t - p_goal);
        distance_to_goal = [distance_to_goal, error1];
        error2 = norm(Rz - R_init);
        difference_axis = [difference_axis, error2];
        distance_to_wall = [distance_to_wall, (t - p_wall)' * n_wall];
        J = J_space(S, q);
        J_alpha = J(1:3, :);
        J_p = J(4:6, :);

        A1 = J_p - vecToso3(t) * J_alpha;
        b1 = p_goal - t;
        A2 = sqrt(eta) * vecToso3(Rz) * J_alpha;
        b2 = -sqrt(eta) * (R_init - Rz);
        % Bounds
        lb = qL - q;
        ub = qU - q;

        options = optimoptions('lsqlin','Display','none');
        % options = optimoptions('lsqlin');
        delta_q_dir = lsqlin([A1; A2], [b1; b2], [], [], [], [], lb, ub, [], options);
        % Line search to find appropriate step size
        alpha = 1.0;  % Reset step size
        best_q = q;
        for i = 1:max_iter
            % Try candidate step
            q_candidate = q + alpha * delta_q_dir;
            
            % Check if candidate violates wall constraint
            Tsb_candidate = FK_space(M, S, q_candidate);
            t_candidate = Tsb_candidate(1:3, 4);
            if norm(t_candidate - p_goal) > norm(t - p_goal)
                alpha = alpha * beta;
                continue;
            else 
                best_q = q_candidate;
                break;
            end
        end
        q = best_q;
        q_history = [q_history, q];
    end
end