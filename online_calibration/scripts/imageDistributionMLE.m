function h = imageDistributionMLE(image, min, max)

% variable declarations
h(min:max) = double(1);
[v, u] = size(image);

% histogram of image
for i=1:u
    for j=1:v
        h(image(j, i)) = h(image(j, i)) + 1;
    end
end

% normalization
h = h / sum(h);

end

