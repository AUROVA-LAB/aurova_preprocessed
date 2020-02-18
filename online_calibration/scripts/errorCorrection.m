function data = errorCorrection (data, data_prep, matches, params)

% image points calculation
image_points = cat(1, matches.kp_src, matches.pair_src);

% world points calculation
image_w_points = cat(1, matches.kp_tmp, matches.pair_tmp);
[mm, ~] = size(image_w_points);
world_points(1:mm, 1:3) = 0;
for i = 1:mm
    v = image_w_points(i, 2);
    u = image_w_points(i, 1);
    world_points(i, 1) = data_prep.image_world(v, u, 1);
    world_points(i, 2) = data_prep.image_world(v, u, 2);
    world_points(i, 3) = data_prep.image_world(v, u, 3);
end

[orientation, location, inliers, status] = estimateWorldCameraPose(image_points, world_points, params.camera_params);

data.orientation = orientation;
data.location = location;
data.inliers = inliers;
data.status = status;

end