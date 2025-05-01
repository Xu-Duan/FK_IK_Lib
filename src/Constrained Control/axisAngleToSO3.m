function R = axisAngleToSO3(axis, angle)
    assert(isequal(size(axis), [3, 1]), 'axis must be a 3x1 matrix.');
    % Normalize the axis vector
    if all(axis == [0; 0; 0])
        R = eye(3);
        return;
    end
    axis = axis / norm(axis);
    
    % Extract components
    x = axis(1);
    y = axis(2);
    z = axis(3);
    
    % Compute trigonometric values
    c = cos(angle);
    s = sin(angle);
    t = 1 - c;

    % Construct the rotation matrix
    % R = [t*x*x + c,   t*x*y - s*z, t*x*z + s*y;
    %      t*x*y + s*z, t*y*y + c,   t*y*z - s*x;
    %      t*x*z - s*y, t*y*z + s*x, t*z*z + c];
    % Compute trigonometric values
    
    % Construct the rotation matrix using Rodrigues' formula
    R = c * eye(3) + s * [0, -z, y; z, 0, -x; -y, x, 0] + t * (axis * axis');
end