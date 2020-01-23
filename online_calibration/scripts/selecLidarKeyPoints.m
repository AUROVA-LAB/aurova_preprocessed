function descriptor = selecLidarKeyPoints(data_prep, params)

% filter depth image in some range
min_pixel = mapDistanceToUint(params.max_distance, params.sigma, params.base);
img_nearest_pt = data_prep.img_depth > min_pixel;

% selec points with certain edge intensity value
min_intensity = mapDistanceToUint(params.min_intensity, params.sigma, params.base);
img_strong_edges = data_prep.img_discnt > min_intensity;

% select final candidates for KP
img_candidates = logical(img_nearest_pt .* img_strong_edges);
[i_y, i_x] = find(logical(img_candidates));
observations = cat(2, i_x, i_y);

% clustering points in K areas
i_k = kmeans(observations, params.k);

% first aproximation: get random values with min-max distance

%**************************************
%**************************************
%debug
[m, n] = size(data_prep.img_depth);
k = length(i_k);
img_cluster(1:m, 1:n) = 0;
for i = 1:k
    img_cluster(i_y(i), i_x(i)) = (i_k(i) / params.k) * params.base;
end
img_cluster = ind2rgb(uint8(img_cluster), jet(params.base+1));
img_cluster(:, :, 1) = img_cluster(:, :, 1) .* img_candidates;
img_cluster(:, :, 2) = img_cluster(:, :, 2) .* img_candidates;
img_cluster(:, :, 3) = img_cluster(:, :, 3) .* img_candidates;

figure
imshow(img_nearest_pt);
figure
imshow(img_strong_edges);
figure
imshow(img_cluster);
%**************************************
%**************************************

descriptor = [];
descriptor.kp = [];
descriptor.pair = [];
descriptor.distance = 0;
descriptor.rotation = 0;
descriptor.roi = [];
descriptor.roi.p11 = [];
descriptor.roi.p12 = [];
descriptor.roi.p21 = [];

end