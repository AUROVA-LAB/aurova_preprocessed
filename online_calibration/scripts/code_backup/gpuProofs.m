function [a, b] = gpuProofs(kp_x, kp_y, pair_x, pair_y, tmp_x, tmp_y, escalar1, escalar2)

a = (kp_x - pair_x) .* tmp_x * escalar1;
b = (kp_y - pair_y) .* tmp_y * escalar2;

end

