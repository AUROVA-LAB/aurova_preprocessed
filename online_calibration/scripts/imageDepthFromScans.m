function image_depth = imageDepthFromScans(scans_mapframe, tfs_map2camera, index, sigma, base, read_mat)

if read_mat
    image_depth = load(num2str(index,'mat_data/image_depth%d.mat'), 'image_depth');
    image_depth = image_depth.image_depth;
else
    camera_params = getCameraParams();
    xy = camera_params.ImageSize;
    image_depth(1:xy(2), 1:xy(1)) = uint8(0);
    m = length(scans_mapframe);
    id =  (m - 1) / 2 + 1;
    tform_map2camera = tfs_map2camera{id};
    for j = 1:m
        scan_cameraframe = pctransform(scans_mapframe{j}, tform_map2camera);
        points_camframe = scan_cameraframe.Location;
        points_camframe = points_camframe(points_camframe(:, 3) > 0, :);

        image_points = worldToImageSimple(camera_params, points_camframe);
        n = length(points_camframe(:, 1));
        for i = 1:n
            u = round(image_points(i, 1));
            v = round(image_points(i, 2));
            if (v >= 1 && v <= xy(2)) && (u >= 1 && u <= xy(1))
                dist_value = points_camframe(i, 3);
                image_depth(v, u) = mapDistanceToUint(dist_value, sigma, base);
            end
        end
    end
    save(num2str(index,'mat_data/image_depth%d.mat'), 'image_depth');
end

end

