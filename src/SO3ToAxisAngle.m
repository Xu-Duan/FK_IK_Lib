function [omega, theta] = SO3ToAxisAngle(R)
    % ROTATIONTOAXISANGLE Convert rotation matrix to axis-angle representation
    %   Input:
    %       R: 3x3 rotation matrix in SO(3)
    %   Output:
    %       axis: 3x1 unit vector representing the axis of rotation
    %       angle: scalar value representing the angle of rotation in radians
    
    % Input validation
    if ~isequal(size(R), [3,3])
        error('Input must be a 3x3 matrix');
    end
    if abs(det(R) - 1) > 1e-10
        error('Matrix must have determinant 1');
    end
    if norm(R*R' - eye(3)) > 1e-10
        error('Matrix must be orthogonal');
    end
    
    % Calculate angle using trace
    theta = acos((trace(R) - 1)/2);
    
    % Handle special cases
    if theta < 1e-10  % Rotation angle close to 0
        omega = [0; 0; 1];  % Default axis when no rotation
        theta = 0;
    elseif abs(theta - pi) < 1e-10  % Rotation angle close to pi
        % Find the largest diagonal element
        [~, idx] = max(diag(R));
        v = zeros(3,1);
        v(idx) = 1;
        % Calculate axis using the corresponding column
        omega = sqrt((R + eye(3))/2) * sign(v);
    else
        % Standard case - use skew symmetric portion
        omega = 1/(2*sin(theta)) * [R(3,2)-R(2,3); 
                                  R(1,3)-R(3,1); 
                                  R(2,1)-R(1,2)];
    end
    
    % Ensure axis is a unit vector
    omega = omega / norm(omega);
end 