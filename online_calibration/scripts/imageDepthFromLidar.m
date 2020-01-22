function [image_depth, image_intsty] = imageDepthFromLidar(scan_lidarframe, data, params)

yx = params.camera_params.image_size;
image_depth(1:yx(1), 1:yx(2)) = uint8(0);
image_intsty(1:yx(1), 1:yx(2)) = uint8(0);
m = scan_lidarframe.Count;
for j = 1:m
    point = scan_lidarframe.Location(j, :);
    intensity = scan_lidarframe.Intensity(j);
    
    point_pc_lid = cat(2, point, 1);
    point_pc_cam = point_pc_lid * data.tf.T;
    
    image_point = worldToImageSimple(params.camera_params, point_pc_cam);

    if point_pc_cam(3) > 0
        u = round(image_point(1));
        v = round(image_point(2));
        if (v >= 1 && v <= yx(1)) && (u >= 1 && u <= yx(2))
            dist_value = point_pc_cam(3);
            image_depth(v, u) = mapDistanceToUint(dist_value, params.sigma, params.base);
            image_intsty(v, u) = intensity;
        end
    end
end

end

