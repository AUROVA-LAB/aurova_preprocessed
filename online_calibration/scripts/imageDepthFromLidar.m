function [image_depth, image_intsty] = imageDepthFromLidar(scan_lidarframe, tf_lidar2map, tf_map2camera, sigma, base)

camera_params = getCameraParams();
xy = camera_params.ImageSize;
image_depth(1:xy(2), 1:xy(1)) = uint8(0);
image_intsty(1:xy(2), 1:xy(1)) = uint8(0);
m = scan_lidarframe.Count;
for j = 1:m
    point = scan_lidarframe.Location(j, :);
    intensity = scan_lidarframe.Intensity(j);
    
    point_pc_lid = pointCloud(point);
    
    point_pc_map = pctransform(point_pc_lid, tf_lidar2map);
    point_pc_cam = pctransform(point_pc_map, tf_map2camera);
    point_camframe = point_pc_cam.Location;
    
    image_point = worldToImageSimple(camera_params, point_camframe);

    if point_camframe(3) > 0
        u = round(image_point(1));
        v = round(image_point(2));
        if (v >= 1 && v <= xy(2)) && (u >= 1 && u <= xy(1))
            dist_value = point_camframe(3);
            image_depth(v, u) = mapDistanceToUint(dist_value, sigma, base);
            image_intsty(v, u) = intensity;
        end
    end
end

end

