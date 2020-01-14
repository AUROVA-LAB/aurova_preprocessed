function [i, j] = gpuProofs2(ii, N)

i = floor((ii-1) / N) + 1;
j = mod(ii-1, N) + 1;

end

