function plot_wall(n_wall, p_wall)
    % plot_wall.m
    % Plot a virtual wall defined by a point and normal vector
    % Inputs:
    %   n_wall: 3x1 normal vector of the wall
    %   p_wall: 3x1 point on the wall

    % Create a grid of points on the wall
    % First, find two vectors perpendicular to n_wall
    [v1, v2] = get_perpendicular_vectors(n_wall);
    
    % Create a grid
    grid_size = 1;  % Size of the wall visualization
    [X, Y] = meshgrid(linspace(-grid_size, grid_size, 10));
    
    % Convert grid points to 3D coordinates on the wall
    Z = zeros(size(X));
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            point = p_wall + X(i,j)*v1 + Y(i,j)*v2;
            Z(i,j) = point(3);  % Z coordinate
            X(i,j) = point(1);  % Update X coordinate
            Y(i,j) = point(2);  % Update Y coordinate
        end
    end
    
    % Plot the wall as a semi-transparent surface
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeAlpha', 0.5, ...
         'FaceColor', [0.8 0.8 1]);  % Light blue color
    hold on;
    
    % Plot normal vector
    quiver3(p_wall(1), p_wall(2), p_wall(3), ...
            n_wall(1), n_wall(2), n_wall(3), 0.1, ...
            'LineWidth', 2, 'Color', 'r');
end

function [v1, v2] = get_perpendicular_vectors(n)
    % Find two vectors perpendicular to n
    if abs(n(3)) < abs(n(1))
        v1 = cross(n, [0; 0; 1]);
    else
        v1 = cross(n, [1; 0; 0]);
    end
    v1 = v1 / norm(v1);
    v2 = cross(n, v1);
    v2 = v2 / norm(v2);
end 