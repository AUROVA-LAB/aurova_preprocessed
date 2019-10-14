function h = histogramLocal(image, min, max)

h(min:max) = double(1);
[v, u] = size(image);

for i=1:u
    for j=1:v
        h(image(j, i)) = h(image(j, i)) + 1;
    end
end

end

