function ellipsoid_plot_angular(J)
% ELLIPSOID_PLOT_ANGULAR Plot the angular velocity manipulability ellipsoid
%
% Input:
%   J - 6xn Jacobian matrix
%
% Description:
%   Plots the angular velocity manipulability ellipsoid, which shows
%   the robot's ability to rotate in different directions.
%   The size and shape of the ellipsoid indicate the robot's
%   angular velocity capabilities.

% Extract angular velocity components (top 3 rows of Jacobian)
J_w = J(1:3, :);

% Calculate the angular velocity manipulability matrix
A = J_w * J_w';

% Get eigenvalues and eigenvectors
[V, D] = eig(A);

% Extract eigenvalues and sort them
d = diag(D);
[d, idx] = sort(d, 'descend');
V = V(:, idx);

% Create unit sphere points
[X, Y, Z] = sphere(50);

% Scale the sphere by eigenvalues
X = X * sqrt(d(1));
Y = Y * sqrt(d(2));
Z = Z * sqrt(d(3));

% Rotate the ellipsoid by eigenvectors
for i = 1:size(X,1)
    for j = 1:size(X,2)
        point = V * [X(i,j); Y(i,j); Z(i,j)];
        X(i,j) = point(1);
        Y(i,j) = point(2);
        Z(i,j) = point(3);
    end
end

% Create new figure
figure;
hold on;

% Plot the ellipsoid
surf(X, Y, Z, 'FaceColor', 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Plot the principal axes
scale = 0.5;  % Scale factor for axes
for i = 1:3
    quiver3(0, 0, 0, V(1,i)*sqrt(d(i))*scale, V(2,i)*sqrt(d(i))*scale, V(3,i)*sqrt(d(i))*scale, ...
            'LineWidth', 2, 'Color', 'r');
end

% Set plot properties
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Angular Velocity Manipulability Ellipsoid');
legend('Ellipsoid', 'Principal Axes');

% Add text showing the volume
volume = sqrt(prod(d));
text(0.1, 0.1, 0.1, sprintf('Volume: %.2f', volume), 'FontSize', 12);

% Set view
view(3);
rotate3d on;

end 