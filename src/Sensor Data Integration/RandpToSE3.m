function T = RandpToSE3(R, p)
T = [R p; zeros(1, 3) 1];
end