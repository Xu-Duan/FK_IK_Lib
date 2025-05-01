function R = quaternionToSO3(q)
    % Normalize the quaternion
    q = q / norm(q);
    
    % Extract components
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % % Construct the rotation matrix
    % R = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*z*w,     2*x*z + 2*y*w;
    %      2*x*y + 2*z*w,     1 - 2*x^2 - 2*z^2, 2*y*z - 2*x*w;
    %      2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x^2 - 2*y^2];

    % Construct the rotation matrix
    R = [w^2 + x^2 - y^2 - z^2, 2*x*y - 2*z*w,     2*x*z + 2*y*w;
         2*x*y + 2*z*w,     w^2 - x^2 + y^2 - z^2, 2*y*z - 2*x*w;
         2*x*z - 2*y*w,     2*y*z + 2*x*w,     w^2 - x^2 - y^2 + z^2];
end