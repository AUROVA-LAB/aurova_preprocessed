clear, clc, close all

%****************** global variables ******************%
index = 250;
window_size = 0;
sigma = 10;
base = 255;
read_mat = [false false false false];
exec_flag = [true true false false];

%************* read and store raw data ****************% T = [30 - 40] s
if exec_flag(1)
    [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera, image] = ...
        readData(read_mat(1), index, window_size);
    
    image_gray = rgb2gray(image);
    %image_gray = imgaussfilt(image_gray, 2.0);
    [image_grad, image_dir] = imgradient(image_gray, 'sobel');
    image_grad = image_grad / max(max(image_grad));
    figure
    imshow(image_grad)
end

%********** project 3D points into 2D pixel plane ********%; T = 5 s
if exec_flag(2)
    image_depth = imageDepthFromScans(scans_mapframe, tfs_map2camera, ...
                                      index, window_size, sigma, base, read_mat(2));
    figure
    imshow(image_depth)
end

%****************** object segmentation ****************% T = 70 s
if exec_flag(3)
    [objects_gray, objects_depth] = imageSegmentation(image, image_depth, index, read_mat(3));
    figure
    montage(objects_gray)
    figure
    montage(objects_depth)
end

%********** objects corregistration scans-image **********% T = 15 s
if exec_flag(4)
    plot_gray = objectsCorregistration(objects_gray, objects_depth, image);
%     figure
%     imshow(plot_gray)
end