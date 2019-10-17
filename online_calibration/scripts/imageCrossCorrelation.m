function map = imageCrossCorrelation(image, template)

% variable declarations
[v, u, c] = size (image);
[m, n, k] = size (template);
map(1:v, 1:u) = double(0); 
image = double(image);
template = double(template);
% template = template - mean(mean(template));

% correlation in all image
for j = 1:v-m
    for i = 1:u-n
        target = image(j:j+m-1, i:i+n-1);
        target = target - mean(mean(target)); % to prevent uniform targets
        map(j, i) = sum(sum(template .* target));
    end
end

% normalization
map = map / max(max(map));

end

