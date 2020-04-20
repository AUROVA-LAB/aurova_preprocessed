function descriptors = selecLidarKeyPoints(data_prep, params)

% crete descriptors struct
descriptors = [];
descriptors.kp = [];
descriptors.pair = [];
descriptors.distance = [];
descriptors.rotation = [];
descriptors.roi = [];
descriptors.roi.p11 = [];
descriptors.roi.p12 = [];
descriptors.roi.p21 = [];
descriptors.roip = [];
descriptors.roip.p11 = [];
descriptors.roip.p12 = [];
descriptors.roip.p21 = [];
descriptors.cluster = {};
descriptors.entropy = [];

% GENERATE CLUSTERS IN Z AXE
n = params.camera_params.ImageSize(2);
m = params.camera_params.ImageSize(1);
num_seg = round(params.lidar_parameters.max_range / params.s);
clusters_z = {};
for i = 1:2 %TODO: parameter
    threshold_up = i*params.s;
    threshold_dw = (i-1)*params.s;
    mask_ia = data_prep.img_depth_real < threshold_up;
    mask_ib = data_prep.img_depth_real > threshold_dw;
    mask_i = mask_ia .* mask_ib;
    discnt = data_prep.img_discnt > params.threshold_dsc;
    cluster_i = discnt .* mask_i;
    N_i = sum(sum(cluster_i));
    if N_i > params.min_sz_clus
        clusters_z = [clusters_z cluster_i];
    end
end

% GENERATE CLUSTERS IN y AXE
clusters_zy = {};
act_zy_cluster = clusters_z{1};
act_zy_cluster(:, :) = 0;
for i = 1:length(clusters_z)
    
    [i_y, i_x] = find(clusters_z{i} > 0);
    u = i_x;
    v = i_y;
    cluster_j = cat(2, u, v);
    
    mx = mean(cluster_j(:,1));
    my = mean(cluster_j(:,2));
    dists = sqrt((u-mx).*(u-mx) + (v-my).*(v-my));
    ind = find(dists == min(dists));
    centroid = u(ind(1));
    
    act_z_cluster = clusters_z{i};
    act_zy_cluster(:, 1:centroid) = act_z_cluster(:, 1:centroid);
    clusters_zy = [clusters_zy act_zy_cluster];
    act_zy_cluster(:, :) = 0;
    act_zy_cluster(:, centroid:end) = act_z_cluster(:, centroid:end);
    clusters_zy = [clusters_zy act_zy_cluster];
    
%     act_z_cluster = clusters_z{i};
%     act_zy_cluster(:, :) = 0;
%     for j = 1:n-margin
%         num_p_slice = sum(sum(act_z_cluster(:, j:j+margin)));
%         num_p_cluster = sum(sum(act_zy_cluster));
%         if num_p_slice > 0
%             act_zy_cluster(:, j:j+margin) = act_z_cluster(:, j:j+margin);
%         elseif num_p_cluster > params.min_sz_clus
%             clusters_zy = [clusters_zy act_zy_cluster];
%             act_zy_cluster(:, :) = 0;
%         else
%             act_zy_cluster(:, :) = 0;
%         end
%     end
end

% GENERATE DESCRIPTORS
for i = 1:length(clusters_zy)
    [i_y, i_x] = find(clusters_zy{i} > 0);
    u = i_x;
    v = i_y;
    cluster_j = cat(2, u, v);

    % keypoint selection
    mx = mean(cluster_j(:,1));
    my = mean(cluster_j(:,2));
    x_p = mx;
    y_p = my;
    
    for j = 1:length(x_p)
    
        dists = sqrt((u-x_p(j)).*(u-x_p(j)) + (v-y_p(j)).*(v-y_p(j)));
        ind_kp = find(dists == min(dists));
        kp = [u(ind_kp(1)) v(ind_kp(1))];

        % pair selection
        dists = sqrt((u-kp(1)).*(u-kp(1)) + (v-kp(2)).*(v-kp(2)));
        ind_pair = find(dists > params.min_rho & dists < params.max_rho);
        pair_ok = length(ind_pair);

        if pair_ok > 0
            descriptors.kp = cat(1, descriptors.kp, kp);

            pair = [u(ind_pair(1)) v(ind_pair(1))];
            descriptors.pair = cat(1, descriptors.pair, pair);

            [modul, rott, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
            descriptors.distance = cat(1, descriptors.distance, modul);
            descriptors.rotation = cat(1, descriptors.rotation, rott);

            min_x = limitValue(kp(1) - params.area/2, 1, n);
            min_y = limitValue(kp(2) - params.area/2, 1, m);
            max_x = limitValue(kp(1) + params.area/2, 1, n);
            max_y = limitValue(kp(2) + params.area/2, 1, m);
            descriptors.roi.p11 = cat(1, descriptors.roi.p11, [min_x min_y]);
            descriptors.roi.p12 = cat(1, descriptors.roi.p12, [max_x min_y]);
            descriptors.roi.p21 = cat(1, descriptors.roi.p21, [min_x max_y]);

            min_x = limitValue(pair(1) - params.area/2, 1, n);
            min_y = limitValue(pair(2) - params.area/2, 1, m);
            max_x = limitValue(pair(1) + params.area/2, 1, n);
            max_y = limitValue(pair(2) + params.area/2, 1, m);
            descriptors.roip.p11 = cat(1, descriptors.roip.p11, [min_x min_y]);
            descriptors.roip.p12 = cat(1, descriptors.roip.p12, [max_x min_y]);
            descriptors.roip.p21 = cat(1, descriptors.roip.p21, [min_x max_y]);

            descriptors.cluster = [descriptors.cluster cluster_j];
        end
    end
end

% % clustering in xy the points in each slice z
% n = params.camera_params.ImageSize(2);
% m = params.camera_params.ImageSize(1);
% for i = 1:params.s
%     [i_y, i_x] = find(data_prep.img_discnt_msk(:,:,i) > 0);
%     observations = cat(2, i_x, i_y);
%     sz_obs = length(i_y);
%     if (sz_obs > params.min_sz_slice)
%         i_k = kmeans(observations, params.k);
%         for j = 1:params.k
%             indx = find(i_k == j);
%             sz_clus = length(indx);
%             if (sz_clus > params.min_sz_clus)
%                 u = i_x(indx);
%                 v = i_y(indx);
%                 cluster_j = cat(2, u, v);
%                 
%                 % keypoint selection
%                 mx = mean(cluster_j(:,1));
%                 my = mean(cluster_j(:,2));
%                 dists = sqrt((u-mx).*(u-mx) + (v-my).*(v-my));
%                 ind_kp = find(dists == min(dists));
%                 kp = [u(ind_kp(1)) v(ind_kp(1))];
%                 
%                 % pair selection
%                 dists = sqrt((u-kp(1)).*(u-kp(1)) + (v-kp(2)).*(v-kp(2)));
%                 ind_pair = find(dists > params.min_rho & dists < params.max_rho);
%                 pair_ok = length(ind_pair);
%                 
%                 if pair_ok > 0
%                     descriptors.kp = cat(1, descriptors.kp, kp);
%                     
%                     pair = [u(ind_pair(1)) v(ind_pair(1))];
%                     descriptors.pair = cat(1, descriptors.pair, pair);
%                     
%                     [modul, rott, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
%                     descriptors.distance = cat(1, descriptors.distance, modul);
%                     descriptors.rotation = cat(1, descriptors.rotation, rott);
%                     
%                     min_x = limitValue(kp(1) - params.area/2, 1, n);
%                     min_y = limitValue(kp(2) - params.area/2, 1, m);
%                     max_x = limitValue(kp(1) + params.area/2, 1, n);
%                     max_y = limitValue(kp(2) + params.area/2, 1, m);
%                     descriptors.roi.p11 = cat(1, descriptors.roi.p11, [min_x min_y]);
%                     descriptors.roi.p12 = cat(1, descriptors.roi.p12, [max_x min_y]);
%                     descriptors.roi.p21 = cat(1, descriptors.roi.p21, [min_x max_y]);
% 
%                     min_x = limitValue(pair(1) - params.area/2, 1, n);
%                     min_y = limitValue(pair(2) - params.area/2, 1, m);
%                     max_x = limitValue(pair(1) + params.area/2, 1, n);
%                     max_y = limitValue(pair(2) + params.area/2, 1, m);
%                     descriptors.roip.p11 = cat(1, descriptors.roip.p11, [min_x min_y]);
%                     descriptors.roip.p12 = cat(1, descriptors.roip.p12, [max_x min_y]);
%                     descriptors.roip.p21 = cat(1, descriptors.roip.p21, [min_x max_y]);
%                     
%                     descriptors.cluster = [descriptors.cluster cluster_j];
%                 end
%             end
%         end
%     end
% end

% if params.k > 1
%     
%     % filter depth image in some range
%     min_pixel = mapDistanceToUint(params.max_distance, params.sigma, params.base);
%     img_nearest_pt = data_prep.img_depth > min_pixel;
% 
%     % selec points with certain edge intensity value
%     min_intensity = params.min_intensity;
%     img_strong_edges = data_prep.img_discnt > min_intensity;
% 
%     % select final candidates for KP
%     img_candidates = logical(img_nearest_pt .* img_strong_edges);
%     [i_y, i_x] = find(logical(img_candidates));
%     observations = cat(2, i_x, i_y);
% 
%     % clustering points in K areas
%     i_k = kmeans(observations, params.k);
% 
%     % get center of clusters as keypoints and pairs
%     [m, n] = size(data_prep.img_depth);
%     for i = 1:params.k
%         pair_ok = false;
%         ii = find(i_k == i);
%         num = length(ii);
%         kp = [i_x(ii(1)) i_y(ii(1))];
%         j = 1;
%         while j <= num
%             [dist, rot, ~] = cartesian2SphericalInDegrees(i_x(ii(j)) - i_x(ii(1)), i_y(ii(j)) - i_y(ii(1)), 0);
%             if dist >= params.min_rho && dist <= params.max_rho
%                 pair = [i_x(ii(j)) i_y(ii(j))];
%                 descriptors.pair = cat(1, descriptors.pair, pair);
%                 descriptors.distance = cat(1, descriptors.distance, dist);
%                 descriptors.rotation = cat(1, descriptors.rotation, rot);
%                 j = num;
%                 pair_ok = true;
%             end
%             j = j + 1;
%         end
%         if pair_ok
%             descriptors.kp = cat(1, descriptors.kp, kp);
% 
%             min_x = limitValue(kp(1) - params.area/2, 1, n);
%             min_y = limitValue(kp(2) - params.area/2, 1, m);
%             max_x = limitValue(kp(1) + params.area/2, 1, n);
%             max_y = limitValue(kp(2) + params.area/2, 1, m);
%             descriptors.roi.p11 = cat(1, descriptors.roi.p11, [min_x min_y]);
%             descriptors.roi.p12 = cat(1, descriptors.roi.p12, [max_x min_y]);
%             descriptors.roi.p21 = cat(1, descriptors.roi.p21, [min_x max_y]);
% 
%             min_x = limitValue(pair(1) - params.area/2, 1, n);
%             min_y = limitValue(pair(2) - params.area/2, 1, m);
%             max_x = limitValue(pair(1) + params.area/2, 1, n);
%             max_y = limitValue(pair(2) + params.area/2, 1, m);
%             descriptors.roip.p11 = cat(1, descriptors.roip.p11, [min_x min_y]);
%             descriptors.roip.p12 = cat(1, descriptors.roip.p12, [max_x min_y]);
%             descriptors.roip.p21 = cat(1, descriptors.roip.p21, [min_x max_y]);
%         end
%     end
% %     len = params.k;
% %     count = 1:len;
% %     jj = 0;
% %     while len > 0
% %         distance_min = n * 2;
% %         i = count(1);
% %         ii = find(i_k == i);
% %         %kp = [round(median(i_x(ii))) round(median(i_y(ii)))];
% %         kp = [i_x(ii(1)) i_y(ii(1))];
% %         for j = count(2:end)
% %             ii = find(i_k == j);
% %             pair = [i_x(ii(1)) i_y(ii(1))];
% %             [distance, ~, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
% %             if distance < distance_min
% %                 distance_min = distance;
% %                 jj = j;
% %             end
% %         end
% %         ii = find(i_k == jj);
% %         pair = [i_x(ii(1)) i_y(ii(1))];
% % 
% %         % fill data structure
% %         descriptors.kp = cat(1, descriptors.kp, kp);
% %         min_x = limitValue(kp(1) - params.area/2, 1, n);
% %         min_y = limitValue(kp(2) - params.area/2, 1, m);
% %         max_x = limitValue(kp(1) + params.area/2, 1, n);
% %         max_y = limitValue(kp(2) + params.area/2, 1, m);
% %         descriptors.roi.p11 = cat(1, descriptors.roi.p11, [min_x min_y]);
% %         descriptors.roi.p12 = cat(1, descriptors.roi.p12, [max_x min_y]);
% %         descriptors.roi.p21 = cat(1, descriptors.roi.p21, [min_x max_y]);
% % 
% %         descriptors.pair = cat(1, descriptors.pair, pair);
% %         min_x = limitValue(pair(1) - params.area/2, 1, n);
% %         min_y = limitValue(pair(2) - params.area/2, 1, m);
% %         max_x = limitValue(pair(1) + params.area/2, 1, n);
% %         max_y = limitValue(pair(2) + params.area/2, 1, m);
% %         descriptors.roip.p11 = cat(1, descriptors.roip.p11, [min_x min_y]);
% %         descriptors.roip.p12 = cat(1, descriptors.roip.p12, [max_x min_y]);
% %         descriptors.roip.p21 = cat(1, descriptors.roip.p21, [min_x max_y]);
% % 
% %         [dist, rot, ~] = cartesian2SphericalInDegrees(pair(1) - kp(1), pair(2) - kp(2), 0);
% %         descriptors.distance = cat(1, descriptors.distance, dist);
% %         descriptors.rotation = cat(1, descriptors.rotation, rot);
% % 
% %         % actualization for next iteration
% %         count(1) = [];
% %         count(count == jj)  = [];
% %         len = length(count);
% %     end
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