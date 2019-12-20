function [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera, image] = readData(filename, index)

scan_filename_base = strcat(filename, 'scan');
tf_filename_base = strcat(filename, 'tf');
image_filename_base = strcat(filename, 'image');
image_filename = strcat(image_filename_base, num2str(index,'%d.jpg'));
scan_filename = strcat(scan_filename_base, num2str(index,'%d.pcd'));
tf_filename = strcat(tf_filename_base, num2str(index,'%d.csv'));
image = imread(image_filename);
scan = pcread(scan_filename);
xyz_rpy = csvread(tf_filename);
tform_lidar2map = getTfAffineMatrix(xyz_rpy, 1);
tform_map2camera = getTfAffineMatrix(xyz_rpy, 2);

scans_lidarframe = {};
scans_mapframe = {};
tfs_lidar2map = {};
tfs_map2camera = {};

%scan = helperProcessPointCloud(scan);
scan_mapframe = pctransform(scan, tform_lidar2map);
tfs_lidar2map = [tfs_lidar2map; {tform_lidar2map}];
tfs_map2camera = [tfs_map2camera; {tform_map2camera}];
scans_lidarframe = [scans_lidarframe; {scan}];
scans_mapframe = [scans_mapframe; {scan_mapframe}];
end

