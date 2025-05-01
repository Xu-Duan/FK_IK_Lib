function [omega_hat] = vecToso3(omega)
assert(length(omega) == 3)
omega_hat = zeros(3); 

omega_hat(1, 2) = -omega(3);
omega_hat(1, 3) = omega(2);
omega_hat(2, 1) = omega(3);
omega_hat(2, 3) = -omega(1);
omega_hat(3, 1) = -omega(2);
omega_hat(3, 2) = omega(1);
end