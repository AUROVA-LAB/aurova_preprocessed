clear, clc, close all

%****************** global variables ******************%
index = 250;
window_size = 0;
sigma = 20;
base = 255;
threshold_dw = 0.0;
threshold_up = 70;
read_mat = [false false false false];
exec_flag = [true true true false];

%************* read and store raw data ****************%
if exec_flag(1)
    [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera, image] = ...
        readData(read_mat(1), index, window_size);
    image_gray = rgb2gray(image);
    [image_grad, image_dir] = imgradient(image_gray, 'sobel');
    image_grad = uint8(image_grad / max(max(image_grad)) * 255);
    figure
    imshow(image_grad)
end

%************** filtering of scan lidar *******************%
if exec_flag(2)
    st_lidar_cfg = fillLidarCfg(scans_lidarframe{1});
    scan_filtered = filterScanAzimuth(scans_lidarframe{1}, st_lidar_cfg, threshold_dw, threshold_up, base);
end

%********** project 3D points into 2D pixel plane ********%
if exec_flag(3)
    [image_depth, image_discnt] = imageDepthFromLidar(scan_filtered, tfs_lidar2map{1}, tfs_map2camera{1}, sigma, base);

    
    image_grad(image_depth > 0) = 255;
    image_grad(image_discnt > 50) = 0;
    image_depth_plt(:, :, 1) = image_grad;
    image_grad(image_depth > 0) = 0;
    image_grad(image_discnt > 50) =  255;
    image_depth_plt(:, :, 2) = image_grad;
    image_grad(image_depth > 0) =  0;
    image_grad(image_discnt > 50) =  0;
    image_depth_plt(:, :, 3) = image_grad;
    
    figure
    imshow(image_depth)
    figure
    imshow(image_discnt)
    figure
    imshow(image_depth_plt)
end





% % BACKUP:
% %****************** object segmentation ****************% T = 70 s
% if exec_flag(3)
%     [objects_gray, objects_depth] = imageSegmentation(image, image_depth, index, read_mat(3));
%     figure
%     montage(objects_gray)
%     figure
%     montage(objects_depth)
% end
% 
% %********** objects corregistration scans-image **********% T = 15 s
% if exec_flag(4)
%     plot_gray = objectsCorregistration(objects_gray, objects_depth, image);
%     figure
%     imshow(plot_gray)
% end