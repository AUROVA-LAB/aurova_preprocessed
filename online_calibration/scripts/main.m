clear, clc, close all

%****************** global variables ******************%
index = 450;
window_size = 300;
sigma = 10;
base = 255;
id_secuence = window_size / 2;

%************* read and store raw data ****************%
is_m_data = true;
[scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera] = ...
    readData(is_m_data, index, window_size);

%********** project 3D points into 2D pixel plane ********%;
is_m_data = true;
image_depth = imageDepthFromScans(scans_mapframe, tfs_map2camera, ...
                                                             id_secuence, sigma, base, is_m_data);

%****************** object segmentation ****************%
is_m_data = true;
[objects_gray, objects_depth] = imageSegmentation(image_depth, index, is_m_data);

%********** objects corregistration scans-image **********%
plot_gray = objectsCorregistration(objects_gray, objects_depth, index);

%*********************** plot *************************%
% figure
% pcshow(cloud, 'MarkerSize', 30);
figure
imshow(image_depth)
figure
montage(objects_gray);
figure
montage(objects_depth);
figure
imshow(plot_gray)
