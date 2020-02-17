function [image_depth, image_intsty, image_world] = imageDepthFromLidar(scan_lidarframe, data, params)

yx = params.camera_params.image_size;
image_depth(1:yx(1), 1:yx(2)) = uint8(0);
image_intsty(1:yx(1), 1:yx(2)) = uint8(0);
image_world(1:yx(1), 1:yx(2), 1:3) = double(0);
m = scan_lidarframe.Count;
for j = 1:m
    point = scan_lidarframe.Location(j, :);
    intensity = scan_lidarframe.Intensity(j);
    
    point_pc_lid = cat(2, point, 1);
    point_pc_cam = point_pc_lid * data.tf.T * data.tf_miss.T;
    
    image_point = worldToImageSimple(params.camera_params, point_pc_cam);
%     cam_params_aux = cameraParameters(...
%     'IntrinsicMatrix', params.calib_aux.K{1}', ...
%     'ImageSize', [512 1392], ...
%     'RadialDistortion', params.calib_aux.D{1}(1:3), ...
%     'TangentialDistortion', params.calib_aux.D{1}(4:5), ...
%     'NumRadialDistortionCoefficient', 3, ...
%     'EstimateTangentialDistortion', true);
%     image_point = worldToImage(cam_params_aux, eye(3), [0 0 0], point_pc_cam(1:3));

    if point_pc_cam(3) > 0
        u = round(image_point(1));
        v = round(image_point(2));
        if (v >= 1 && v <= yx(1)) && (u >= 1 && u <= yx(2))
            dist_value = point_pc_cam(3);
            image_depth(v, u) = mapDistanceToUint(dist_value, params.sigma, params.base);
            image_intsty(v, u) = intensity;
            image_world(v, u, 1) = point_pc_cam(1);
            image_world(v, u, 2) = point_pc_cam(2);
            image_world(v, u, 3) = point_pc_cam(3);
        end
    end
end

end

