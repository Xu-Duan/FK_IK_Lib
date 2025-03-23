function [S, theta] = SE3ToTwist(T)
assert(isequal(size(T), [4, 4]), 'T must be a 4x4 matrix.');
R = zeros(3,3); 
p = zeros(3,1);

R(1:3,1:3) = T(1:3,1:3);
p(1:3) = T(1:3,4);

[omega, theta] = SO3ToAxisAngle(R);

function G_inv = G_inv_theta(omega, theta)
    % Computes the inverse of the G(theta) matrix for axis-angle representation.
    %
    % Inputs:
    %   omega - 3x1 unit vector representing the rotation axis
    %   theta - Rotation angle in radians
    %
    % Output:
    %   G_inv - 3x3 matrix (inverse of G(theta))

    % Validate input size
    if size(omega,1) ~= 3 || size(omega,2) ~= 1
        error('w must be a 3x1 column vector.');
    end
    
    % Ensure w is a unit vector
    if abs(norm(omega) - 1) > 1e-6
        error('w must be a unit vector (norm should be 1).');
    end

    % Identity matrix
    I = eye(3);

    % Compute the skew-symmetric matrix of w
    w_skew = vecToso3(omega);

    % Handle singularity at theta ≈ 0
    if abs(theta) < 1e-6
        G_inv = I - 0.5 * w_skew; % Approximation for small theta
    else
        % Compute G⁻¹(θ) using the given formula
        G_inv = (1/theta) * I - (1/2) * w_skew + ( (1/theta - (1/2) * cot(theta / 2)) * (w_skew^2) );
    end
end

G_inv = G_inv_theta(omega, theta);

v= G_inv * p;
S = [omega; v];
S = S * theta;
end