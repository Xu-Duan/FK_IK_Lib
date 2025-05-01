function [q_history, distance_to_goal, difference_axis, distance_to_wall] = opti_ik_angle_wall_visualize(M, S, p_goal, q_init, qL, qU, n_wall, p_wall)
    % opti_ik_angle_wall_visualize.m
    % This function performs inverse kinematics using the space Jacobian
    % with a virtual wall constraint and minimizing the change in end-effector axis.
    
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
    epsilon1 = 0.003;  % Position error threshold
    epsilon2 = 0.01;    % Axis orientation error threshold
    z_s = [0; 0; 1];  % End-effector axis direction in space frame
    eta = 0.1;        % Weight for axis orientation objective

    q_history = [q_init];
    q = q_init;
    distance_to_goal = [];
    difference_axis = [];
    distance_to_wall = [];
    error1 = 1;
    error2 = 1;

    % Get initial end-effector axis orientation
    Tsb = FK_space(M, S, q);
    R_init = Tsb(1:3, 1:3) * z_s;

    % Line search parameters
    alpha = 1.0;  % Initial step size
    beta = 0.5;   % Step size reduction factor
    max_iter = 20; % Maximum line search iterations
    
    while error1 > epsilon1 || error2 > epsilon2
        % Forward kinematics: Calculate current end-effector configuration
        Tsb = FK_space(M, S, q);
        t = Tsb(1:3, 4);
        Rz = Tsb(1:3, 1:3) * z_s;
        
        distance_to_wall = [distance_to_wall, (t - p_wall)' * n_wall];
        % Calculate errors
        error1 = norm(t - p_goal);
        error2 = norm(Rz - R_init);
        distance_to_goal = [distance_to_goal, error1];
        difference_axis = [difference_axis, error2];
        % Calculate Jacobian
        J = J_space(S, q);
        J_alpha = J(1:3, :);
        J_p = J(4:6, :);

        % Main task: reach target position
        A1 = J_p - vecToso3(t) * J_alpha;
        b1 = p_goal - t;
        
        % Secondary task: minimize axis orientation change
        A2 = sqrt(eta) * vecToso3(Rz) * J_alpha;
        b2 = -sqrt(eta) * (R_init - Rz);
        
        % Wall constraint as inequality constraint
        % We want: (t + A1 * delta_q - p_wall)' * n_wall >= 0
        A_wall = -n_wall' * A1;  % Negative because lsqlin uses <= form
        b_wall = (t - p_wall)' * n_wall;  % Current distance to wall
        % b_wall
        % error1
        % Joint limits
        lb = qL - q;
        ub = qU - q;

        % Solve constrained optimization problem
        options = optimoptions('lsqlin','Display','none');
        delta_q_dir = lsqlin([A1; A2], [b1; b2], A_wall, b_wall, [], [], lb, ub, [], options);
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
        % Update joint angles
        q = best_q;
        q_history = [q_history, q];
    end
end