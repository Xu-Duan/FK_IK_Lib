function [omega] = so3ToVec(omega_hat)
    assert(all(size(omega_hat) == [3, 3]))
    assert(omega_hat(1, 1) == 0)
    assert(omega_hat(2, 2) == 0)
    assert(omega_hat(3, 3) == 0)
    omega(1) = omega_hat(3, 2);
    omega(2) = omega_hat(1, 3);
    omega(3) = omega_hat(2, 1);
end