function descriptor = selecLidarKeyPoints(data_prep, params)

% filter depth image in some range
min_pixel = mapDistanceToUint(params.max_distance, params.sigma, params.base);
img_nearest_pt = data_prep.img_depth > min_pixel;

% selec points with certain edge intensity value
min_intensity = params.min_intensity;
img_strong_edges = data_prep.img_discnt > min_intensity;

% select final candidates for KP
img_candidates = logical(img_nearest_pt .* img_strong_edges);
[i_y, i_x] = find(logical(img_candidates));
observations = cat(2, i_x, i_y);

% clustering points in K areas
i_k = kmeans(observations, params.k);

% crete descriptor struct
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
% get center of clusters as keypoints and pairs
[m, n] = size(data_prep.img_depth);

len = params.k;
count = 1:len;
jj = 0;
while len > 0
    distance_min = n * 2;
    i = count(1);
    ii = find(i_k == i);
    kp = [round(median(i_x(ii))) round(median(i_y(ii)))];
    for j = count(2:end)
        ii = find(i_k == j);
        pair = [round(median(i_x(ii))) round(median(i_y(ii)))];
        [distance, ~, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
        if distance < distance_min
            distance_min = distance;
            jj = j;
        end
    end
    ii = find(i_k == jj);
    pair = [round(median(i_x(ii))) round(median(i_y(ii)))];
    
    % fill data structure
    descriptor.kp = cat(1, descriptor.kp, kp);
    min_x = limitValue(kp(1) - params.area/2, 1, n);
    min_y = limitValue(kp(2) - params.area/2, 1, m);
    max_x = limitValue(kp(1) + params.area/2, 1, n);
    max_y = limitValue(kp(2) + params.area/2, 1, m);
    descriptor.roi.p11 = cat(1, descriptor.roi.p11, [min_x min_y]);
    descriptor.roi.p12 = cat(1, descriptor.roi.p12, [max_x min_y]);
    descriptor.roi.p21 = cat(1, descriptor.roi.p21, [min_x max_y]);

    descriptor.pair = cat(1, descriptor.pair, pair);
    min_x = limitValue(pair(1) - params.area/2, 1, n);
    min_y = limitValue(pair(2) - params.area/2, 1, m);
    max_x = limitValue(pair(1) + params.area/2, 1, n);
    max_y = limitValue(pair(2) + params.area/2, 1, m);
    descriptor.roip.p11 = cat(1, descriptor.roip.p11, [min_x min_y]);
    descriptor.roip.p12 = cat(1, descriptor.roip.p12, [max_x min_y]);
    descriptor.roip.p21 = cat(1, descriptor.roip.p21, [min_x max_y]);
    
    [dist, rot, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
    descriptor.distance = cat(1, descriptor.distance, dist);
    descriptor.rotation = cat(1, descriptor.rotation, rot);
    
    % actualization for next iteration
    count(1) = [];
    count(count == jj)  = [];
    len = length(count);
end

% for i = 1:2:params.k
%     
%     ii = find(i_k == i);
%     kp = [round(median(i_x(ii))) round(median(i_y(ii)))];
%     ii = find(i_k == i + 1);
%     pair = [round(median(i_x(ii))) round(median(i_y(ii)))];
%     
%     descriptor.kp = cat(1, descriptor.kp, kp);
%     min_x = limitValue(kp(1) - params.area/2, 1, n);
%     min_y = limitValue(kp(2) - params.area/2, 1, m);
%     max_x = limitValue(kp(1) + params.area/2, 1, n);
%     max_y = limitValue(kp(2) + params.area/2, 1, m);
%     descriptor.roi.p11 = cat(1, descriptor.roi.p11, [min_x min_y]);
%     descriptor.roi.p12 = cat(1, descriptor.roi.p12, [max_x min_y]);
%     descriptor.roi.p21 = cat(1, descriptor.roi.p21, [min_x max_y]);
% 
%     descriptor.pair = cat(1, descriptor.pair, pair);
%     min_x = limitValue(pair(1) - params.area/2, 1, n);
%     min_y = limitValue(pair(2) - params.area/2, 1, m);
%     max_x = limitValue(pair(1) + params.area/2, 1, n);
%     max_y = limitValue(pair(2) + params.area/2, 1, m);
%     descriptor.roip.p11 = cat(1, descriptor.roip.p11, [min_x min_y]);
%     descriptor.roip.p12 = cat(1, descriptor.roip.p12, [max_x min_y]);
%     descriptor.roip.p21 = cat(1, descriptor.roip.p21, [min_x max_y]);
%     
%     [dist, rot, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
%     descriptor.distance = cat(1, descriptor.distance, dist);
%     descriptor.rotation = cat(1, descriptor.rotation, rot);
%     
% end


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