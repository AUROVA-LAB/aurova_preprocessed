function map = crossCorrelation(image, template)

if size(image,3) == 3
    image = rgb2gray(image);
end

if size(template,3) == 3
    template = rgb2gray(template);
end

[v, u] = size (image);
[m, n] = size (template);
map(1:v, 1:u) = double(0); 
image = double(image);
template = double(template);

for j = 1:v-m
    for i = 1:u-n
        target = image(j:j+m-1, i:i+n-1);
        map(j, i) = sum(sum(template .* target));
    end
end

map = map / max(max(map));

end

