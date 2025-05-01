function q = SO3ToQuaternion(R)
    % This function converts a rotation matrix R ∈ SO(3) to a quaternion [w, x, y, z].

    % Validate that R is a proper rotation matrix

    if ~isequal(size(R), [3,3])
        error('Input must be a 3x3 matrix');
    end
    if abs(det(R) - 1) > 1e-10
        error('Matrix must have determinant 1');
    end
    if norm(R*R' - eye(3)) > 1e-10
        error('Matrix must be orthogonal');
    end
    
    % Compute the trace of R
    tr = trace(R);

    q0= 1/2 * sqrt(tr + 1);

    function p = sgn(x)
        if x < 0
            p = -1;
        else
            p = 1;
        end

    end

    q1 = 1/2 * sgn(R(3,2) - R(2,3)) * sqrt(R(1,1)-R(2,2)-R(3,3)+1);
    q2 = 1/2 * sgn(R(1,3) - R(3,1)) * sqrt(R(2,2)-R(1,1)-R(3,3)+1);
    q3 = 1/2 * sgn(R(2,1) - R(1,2)) * sqrt(R(3,3)-R(2,2)-R(1,1)+1);

    % Return quaternion as [q0, q1, q2, q3]
    q = [q0, q1, q2, q3];

end