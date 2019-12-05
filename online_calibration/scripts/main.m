clear, clc, close all

%****************** global variables ******************%
is_m_data = true;
index = 450;
window_size = 300;

%************* read and store raw data ****************%
[scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera] = ...
    readData(is_m_data, index, window_size);

%********** project 3D points into 2D pixel plane ********%;
camera_params = getCameraParams();
xy = camera_params.ImageSize;
image_cloud(1:xy(2), 1:xy(1)) = uint8(0);
sigma = 10;
base = 255;
m = length(scans_mapframe);
id_secuence = window_size / 2;
tform_map2camera = tfs_map2camera{id_secuence};
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
            image_cloud(v, u) = mapDistanceToUint(dist_value, sigma, base);
        end
    end
end

%*********************** plot *************************%
% figure
% pcshow(cloud, 'MarkerSize', 30);
figure
imshow(image_cloud)
