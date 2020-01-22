function [data, params] = readFileData(params, experiments)

data = [];

if experiments.is_kitti
    
    % selection of camera and paths
    camera = experiments.kitti_cam_id;
    base_dir = experiments.base_dir_cell{experiments.id_dataset};
    calib_dir = experiments.calib_dir_cell{experiments.id_dataset};
    index = experiments.id_sample;

    % load kitti image
    data.image = imread(sprintf('%s/image_%02d/data/%010d.png', base_dir, camera, index));

    % load kitti velodyne points
    file = fopen(sprintf('%s/velodyne_points/data/%010d.bin', base_dir, index), 'rb');
    lidar = fread(file, [4 inf], 'single')';
    fclose(file);
    data.scan = pointCloud(lidar(:, 1:3));
    [n, ~] = size(lidar);
    intensity(1:n) = 0;
    data.scan.Intensity = intensity';
    
    % load transform cam -> lidar
    data.tf = affine3d;
    data.tf.T = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'))';

    % load and compute projection matrix lidar->image plane
    calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
    R_cam_to_rect = eye(4);
    R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
    K_rect = calib.P_rect{camera+1} * R_cam_to_rect;

    % camera parameters
    params.camera_params = [];
    [m, n, ~] = size(data.image); 
    params.camera_params.image_size = [m n];
    params.camera_params.intrinsic_matrix = K_rect ; %calib.K{camera+1}
    
    % lidar parameters
    params.lidar_model = params.id_kitti64;
    params.lidar_parameters = [];
    params.lidar_parameters.res_azimuth = 0.18;
    params.lidar_parameters.max_range = 100.0;
else
    
    % selection of paths
    filename = experiments.filenames_cell{experiments.id_dataset};
    index = experiments.id_sample;
    
    scan_filename_base = strcat(filename, 'scan');
    tf_filename_base = strcat(filename, 'tf');
    image_filename_base = strcat(filename, 'image');
    image_filename = strcat(image_filename_base, num2str(index,'%d.jpg'));
    scan_filename = strcat(scan_filename_base, num2str(index,'%d.pcd'));
    tf_filename = strcat(tf_filename_base, num2str(index,'%d.csv'));
    xyz_rpy = csvread(tf_filename);
    tf_lidar2map = getTfAffineMatrix(xyz_rpy, 1);
    tf_map2camera = getTfAffineMatrix(xyz_rpy, 2);
    data.tf = affine3d;

    data.scan = pcread(scan_filename);
    data.tf.T = tf_lidar2map.T * tf_map2camera.T;
    data.image = imread(image_filename);

    params.camera_params = [];
    [m, n, ~] = size(data.image); 
    params.camera_params.image_size = [m n];
    % TODO: get from file saved in ros!!!
    params.camera_params.intrinsic_matrix = ...
        [461.05267333984375                     0.0                              318.204833984375                    0.0; ...   
         0.0                                    461.2838134765625  186.91806030273438                                0.0; ...
         0.0                                    0.0                              1.0                                 0.1];

    % lidar parameters
    params.lidar_model = params.id_vlp16;
    params.lidar_parameters = [];
    params.lidar_parameters = fillVlp16PuckCfg(data.scan);
end

end

