%%
M = [0 1 0 88; 1 0 0 0; 0 0 -1 926; 0 0 0 1];
omega = [[0; 0; 1], [0; 1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1], [0; -1; 0], [0; 0; 1]];
r = [[0; 0; 10], [0; 0; 333], [0; 0; 649], [82.5; 0; 649], [0; 0; 277], [0; 0; 1033], [88; 0; 926]];
v = zeros(size(omega));
for i = 1:7
    v(:, i) = cross(r(:, i), omega(:, i));
end
S = [omega; v];
B = adjointSE3(inv(M)) * S;
%%
theta = [0; 0; 0; 0; 0; 0; 0];
J_b = J_body(B, theta)
rank(J_b)
J_s = J_space(S, theta)
rank(J_s)
jsingu(J_s)
%%
theta = [0; 0.5; 0; 0; 0; 0; 0];
J_b = J_body(B, theta)
rank(J_b)
J_s = J_space(S, theta)
rank(J_s)
