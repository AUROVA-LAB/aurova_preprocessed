function entropy = clusterEntropy(x, y, grid)

max_x = max(x);
min_x = min(x);
max_y = max(y);
min_y = min(y);

grid_x = round((max_x - min_x) / grid);
grid_y = round((max_y - min_y) / grid);

distribution(1:grid_y+1, 1:grid_x+1) = 0;
entropy = 0;

n = length(x);

for i = 1:n
    u = (x(i) - min_x) / (max_x - min_x);
    u = round(u * grid_x) + 1;
    v = (y(i) - min_y) / (max_y - min_y);
    v = round(v * grid_y) + 1;
    distribution(v, u) = distribution(v, u) + 1;
end

distribution = distribution / sum(sum(distribution));

for i = 1:grid_x+1
    for j = 1:grid_y+1
        if distribution(j, i) > 0
            entropy = entropy - distribution(j, i) * log2(distribution(j, i));
        end
    end
end

end

