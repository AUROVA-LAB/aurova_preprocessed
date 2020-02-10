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
% 1) get random point in a cluster (save in struct)
% 2) get first point in max-min range (save in struct)
% 3) save search area in struct;
[m, n] = size(data_prep.img_depth);
descriptor = [];
descriptor.kp = [];
descriptor.pair = [];
descriptor.distance = [];
descriptor.rotation = [];
descriptor.roi = [];
descriptor.roi.p11 = [];
descriptor.roi.p12 = [];
descriptor.roi.p21 = [];
descriptor.roip = [];
descriptor.roip.p11 = [];
descriptor.roip.p12 = [];
descriptor.roip.p21 = [];
for i = 1:params.k
    pair_ok = false;
    ii = find(i_k == i);
    num = length(ii);
    kp = [i_x(ii(1)) i_y(ii(1))];
    j = 1;
    while j <= num
        [dist, rot, ~] = cartesian2SphericalInDegrees(i_x(ii(j)) - i_x(ii(1)), i_y(ii(j)) - i_y(ii(1)), 0);
        if dist >= params.min_rho && dist <= params.max_rho
            pair = [i_x(ii(j)) i_y(ii(j))];
            descriptor.pair = cat(1, descriptor.pair, pair);
            descriptor.distance = cat(1, descriptor.distance, dist);
            descriptor.rotation = cat(1, descriptor.rotation, rot);
            j = num;
            pair_ok = true;
        end
        j = j + 1;
    end
    if pair_ok
        descriptor.kp = cat(1, descriptor.kp, kp);
        
        min_x = limitValue(kp(1) - params.area/2, 1, n);
        min_y = limitValue(kp(2) - params.area/2, 1, m);
        max_x = limitValue(kp(1) + params.area/2, 1, n);
        max_y = limitValue(kp(2) + params.area/2, 1, m);
        descriptor.roi.p11 = cat(1, descriptor.roi.p11, [min_x min_y]);
        descriptor.roi.p12 = cat(1, descriptor.roi.p12, [max_x min_y]);
        descriptor.roi.p21 = cat(1, descriptor.roi.p21, [min_x max_y]);
        
        min_x = limitValue(pair(1) - params.area/2, 1, n);
        min_y = limitValue(pair(2) - params.area/2, 1, m);
        max_x = limitValue(pair(1) + params.area/2, 1, n);
        max_y = limitValue(pair(2) + params.area/2, 1, m);
        descriptor.roip.p11 = cat(1, descriptor.roip.p11, [min_x min_y]);
        descriptor.roip.p12 = cat(1, descriptor.roip.p12, [max_x min_y]);
        descriptor.roip.p21 = cat(1, descriptor.roip.p21, [min_x max_y]);
    end
end


%**************************************
%**************************************
%debug
% k = length(i_k);
% img_cluster(1:m, 1:n) = 0;
% for i = 1:k
%     img_cluster(i_y(i), i_x(i)) = (i_k(i) / params.k) * params.base;
% end
% img_cluster = ind2rgb(uint8(img_cluster), jet(params.base+1));
% img_cluster(:, :, 1) = img_cluster(:, :, 1) .* img_candidates; %delete background.
% img_cluster(:, :, 2) = img_cluster(:, :, 2) .* img_candidates;
% img_cluster(:, :, 3) = img_cluster(:, :, 3) .* img_candidates;
% 
% figure
% imshow(img_nearest_pt);
% figure
% imshow(img_strong_edges);
% figure
% imshow(img_cluster);
%**************************************
%**************************************

end