clear, clc, close all

%************* global variables *************%
index = 1;
window_size = 800;
scan_filename_base = 'images/input/raw_data_01/scan';
tf_filename_base = 'images/input/raw_data_01/tf';


%** transformation and acumulation scans ***%
step = 5;
for n = index : step : index+window_size
    
    scan_filename = strcat(scan_filename_base, num2str(n,'%d.pcd'));
    tf_filename = strcat(tf_filename_base, num2str(n,'%d.csv'));
    scan = pcread(scan_filename);
    xyz_rpy = csvread(tf_filename);
    tform_lidar2map = getTfAffineMatrix(xyz_rpy, 1);

    scan = helperProcessPointCloud(scan);
    scan = pctransform(scan, tform_lidar2map);
    
    if n == index
        cloud = scan;
    else
        %tform_icp = pcregistericp(scan, cloud); % try ndt and cpd
        %scan = pctransform(scan, tform_icp);
        cloud = pcmerge(cloud, scan, 0.01);
    end
end

cloud = helperProcessPointCloud(cloud);
pcshow(cloud, 'MarkerSize', 30);