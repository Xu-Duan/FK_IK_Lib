function [q_history, distance_to_goal, difference_axis, distance_to_wall] = opti_ik_wall_visualize(M, S, p_goal, q_init, qL, qU, n_wall, p_wall)
    % opti_ik_wall_visualize.m
    % This function performs inverse kinematics using the space Jacobian
    % with a virtual wall constraint and line search.
    
    % Inputs:
    %   M: 4x4 transformation matrix of the end-effector
    %   S: 6xN matrix of normalized twists
    %   p_goal: 3x1 desired position of the end-effector
    %   q_init: 1xN initial guess for the joint angles
    %   qL: Nx1 lower joint limits
    %   qU: Nx1 upper joint limits
    %   n_wall: 3x1 normal vector of the virtual wall (pointing away from valid region)
    %   p_wall: 3x1 point on the virtual wall

    % Input validation
    assert(isequal(size(S, 1), 6), 'S must be a 6xn matrix');
    n = size(S, 2);
    assert(isequal(size(q_init), [n 1]), 'q_init must be nx1');
    assert(isequal(size(M), [4 4]), 'M must be 4x4');
    assert(isequal(size(p_goal), [3 1]), 'p_goal must be 3x1');
    assert(isequal(size(qL), [n 1]), 'qL must be nx1');
    assert(isequal(size(qU), [n 1]), 'qU must be nx1');
    assert(isequal(size(n_wall), [3 1]), 'n_wall must be 3x1');
    assert(isequal(size(p_wall), [3 1]), 'p_wall must be 3x1');

    % Parameters
    epsilon = 0.003;  % Position error threshold
    q_history = [q_init];
    q = q_init;
    distance_to_goal = [];
    distance_to_wall = [];
    difference_axis = [];
    error = 1;
    Tsb = FK_space(M, S, q);
    z_s = [0; 0; 1];
    R_init = Tsb(1:3, 1:3) * z_s;
    % Line search parameters
    alpha = 1.0;  % Initial step size
    beta = 0.5;   % Step size reduction factor
    max_iter = 20; % Maximum line search iterations

    while error > epsilon
        % Forward kinematics: Calculate current end-effector configuration
        Tsb = FK_space(M, S, q);
        t = Tsb(1:3, 4);
        distance_to_wall = [distance_to_wall, (t - p_wall)' * n_wall];
        Rz = Tsb(1:3, 1:3) * z_s;
        difference_axis = [difference_axis, norm(Rz - R_init)];
        % Calculate position error
        error = norm(t - p_goal);
        distance_to_goal = [distance_to_goal, error];
        % Calculate Jacobian
        J = J_space(S, q);
        J_alpha = J(1:3, :);
        J_p = J(4:6, :);

        % Main task: reach target position
        A = J_p - vecToso3(t) * J_alpha;
        b = p_goal - t;
        
        % Wall constraint as inequality constraint
        A_wall = -n_wall' * A;  % Negative because lsqlin uses <= form
        b_wall = (t - p_wall)' * n_wall;  % Current distance to wall
        % b_wall
        % Joint limits
        lb = qL - q;
        ub = qU - q;

        % Solve constrained optimization problem for direction
        options = optimoptions('lsqlin','Display','none');
        delta_q_dir = lsqlin(A, b, A_wall, b_wall, [], [], lb, ub, [], options);
        
        % Line search to find appropriate step size
        alpha = 1.0;  % Reset step size
        best_q = q;
        
        for i = 1:max_iter
            % Try candidate step
            q_candidate = q + alpha * delta_q_dir;
            
            % Check if candidate violates wall constraint
            Tsb_candidate = FK_space(M, S, q_candidate);
            t_candidate = Tsb_candidate(1:3, 4);
            wall_dist = (t_candidate - p_wall)' * n_wall;
            if wall_dist < 0
                alpha = alpha * beta;
                continue;
            else 
                best_q = q_candidate;
                break;
            end
        end
        
        % Update joint angles with best found solution
        q = best_q;
        q_history = [q_history, q];
    end
end