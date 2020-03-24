function rms = computeRms(x_tmp, y_tmp, x_src, y_src, threshold)

n = length(x_tmp);
rms = 0;
P = cat(2, x_src, y_src);

for i = 1:n
    
    p = [x_tmp(i) y_tmp(i)];
    [~, min_dist] = dsearchn(P, p);
    
    if min_dist > threshold
        min_dist = threshold;
    end
    
    rms = rms + min_dist^2;
end

end

