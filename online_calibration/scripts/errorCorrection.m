function data = errorCorrection (data, params)

data.output = [];

if data.matches.num >= data.matches.max
    
    % image points calculation
    image_points = data.matches.kp_src;

    % world points calculation
    world_points = data.matches.wp_tmp;

    [orientation, location, inliers, status] = estimateWorldCameraPose(image_points, world_points, params.camera_params);

    data.output = [];
    data.output.orientation = orientation;
    data.output.location = location;
    data.output.inliers = inliers;
    data.output.status = status;

end

end