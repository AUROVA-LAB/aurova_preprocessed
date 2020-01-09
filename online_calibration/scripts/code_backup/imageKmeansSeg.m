function image_seg = imageKmeansSeg(image, k)

[y, x, c] = size(image);
image = double(image);
image_seg(1:y, 1:x) = uint8(0);
observations(1:y*x, 1:c) = double(0);
index_x(1:y*x, 1) = 0;
index_y(1:y*x, 1) = 0;

n = 1;
for j = 1:y
    for i = 1:x
        observations(n, 1) = image(j, i, 1);
        observations(n, 2) = image(j, i, 2);
        observations(n, 3) = image(j, i, 3);
        index_x(n) = i;
        index_y(n) = j;
        n = n + 1;
    end
end

id_obs = kmeans(observations, k);

for n = 1:length(id_obs)
    image_seg(index_y(n), index_x(n)) = id_obs(n);
end

% image_seg = image_seg * (255 / k);
% image_seg = ind2rgb(uint8(image_seg), jet(256));

end

