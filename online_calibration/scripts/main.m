clear, clc, close all

%****************** global variables ******************%
index = 450;
window_size = 300;
scan_filename_base = 'images/input/raw_data_01/scan';
tf_filename_base = 'images/input/raw_data_01/tf';

%******** transform (to map) and acumulation *********%
scans_sec = {};
step = -1;
ini = index + window_size/2;
endl = index - window_size/2;
for n = ini:step:endl
    
    scan_filename = strcat(scan_filename_base, num2str(n,'%d.pcd'));
    tf_filename = strcat(tf_filename_base, num2str(n,'%d.csv'));
    scan = pcread(scan_filename);
    xyz_rpy = csvread(tf_filename);
    tform_lidar2map = getTfAffineMatrix(xyz_rpy, 1);
    if n == index
        tform_map2camera = getTfAffineMatrix(xyz_rpy, 2);
    end

    scan = helperProcessPointCloud(scan);
    scan = pctransform(scan, tform_lidar2map);
    scans_sec = [scans_sec; {scan}];
    
end
% cloud = helperProcessPointCloud(cloud);



%********** project 3D points into 2D pixel plane ********%;
camera_params = getCameraParams();
xy = camera_params.ImageSize;
image_cloud(1:xy(2), 1:xy(1)) = uint8(0);
sigma = 10;
base = 255;
m = length(scans_sec);
for j = 1:m
    scans_sec{j} = pctransform(scans_sec{j}, tform_map2camera);
    camframe_pts = scans_sec{j}.Location;
    camframe_pts = camframe_pts(camframe_pts(:, 3) > 0, :);
    image_points = worldToImageSimple(camera_params, camframe_pts);

    n = length(camframe_pts(:, 1));
    for i = 1:n
        u = round(image_points(i, 1));
        v = round(image_points(i, 2));
        if (v >= 1 && v <= xy(2)) && (u >= 1 && u <= xy(1))
            dist_value = camframe_pts(i, 3);
            image_cloud(v, u) = mapDistanceToUint(dist_value, sigma, base);
        end
    end
end

%*********************** plot *************************%
% figure
% pcshow(cloud, 'MarkerSize', 30);
figure
imshow(image_cloud)
