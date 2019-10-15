function h = normalizedHistogramMLE(image, min, max)

h(min:max) = double(0);
[v, u] = size(image);

for i=1:u
    for j=1:v
        h(image(j, i)) = h(image(j, i)) + 1;
    end
end

h = h / sum(h);

end

