function [scan_lidarframe, tf_lidar2cam, image, camera_params] = readDataAurova(filename, index)

scan_filename_base = strcat(filename, 'scan');
tf_filename_base = strcat(filename, 'tf');
image_filename_base = strcat(filename, 'image');
image_filename = strcat(image_filename_base, num2str(index,'%d.jpg'));
scan_filename = strcat(scan_filename_base, num2str(index,'%d.pcd'));
tf_filename = strcat(tf_filename_base, num2str(index,'%d.csv'));
xyz_rpy = csvread(tf_filename);
tf_lidar2map = getTfAffineMatrix(xyz_rpy, 1);
tf_map2camera = getTfAffineMatrix(xyz_rpy, 2);
tf_lidar2cam = affine3d;

scan_lidarframe = pcread(scan_filename);
tf_lidar2cam.T = tf_lidar2map.T * tf_map2camera.T;
image = imread(image_filename);

camera_params = [];
[m, n, c] = size(image); 
camera_params.image_size_ = [m n];
camera_params.intrinsic_matrix_ = ...
    [461.05267333984375      0.0                              318.204833984375       0.0; ...  % TODO: get from file saved in ros!!! 
     0.0                                    461.2838134765625  186.91806030273438   0.0; ...
     0.0                                    0.0                              1.0                                 0.1];

end

