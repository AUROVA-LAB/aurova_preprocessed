function [data, params] = readFileData(data, params, experiments)

if experiments.is_kitti(experiments.id_dataset)
    
    % selection of camera and paths
    camera = experiments.kitti_cam_id;
    base_dir = experiments.base_dir_cell{experiments.id_dataset};
    calib_dir = experiments.calib_dir_cell{experiments.id_dataset};
    index = experiments.id_sample;

    % load kitti image
    data.input.image = imread(sprintf('%s/image_%02d/data/%010d.png', base_dir, camera, index));

    % load kitti velodyne points
    file = fopen(sprintf('%s/velodyne_points/data/%010d.bin', base_dir, index), 'rb');
    lidar = fread(file, [4 inf], 'single')';
    fclose(file);
    data.input.scan = pointCloud(lidar(:, 1:3));
    [n, ~] = size(lidar);
    intensity(1:n) = 0;
    data.input.scan.Intensity = intensity';
    
    
    % load each camera calibration parameters 
    calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
    
    % load transform lidar -> camera_xx
    data.input.tf = affine3d;
    tf_velo = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'))';
    tf_cam = eye(4);
    tf_cam(1:3, 1:3) = calib.R{camera+1};
    tf_cam(1:3, 4) = calib.T{camera+1};
    tf_cam = tf_cam';
    tf_rect = eye(4);
    tf_rect(1:3, 1:3) = calib.R_rect{camera+1};
    tf_rect = tf_rect';
    data.input.tf.T = tf_velo * tf_cam * tf_rect;
    

    % camera parameters
    [m, n, ~] = size(data.input.image);
    params.camera_params = cameraParameters(...
                                                 'IntrinsicMatrix', calib.P_rect{camera+1}(1:3, 1:3)', ...
                                                 'ImageSize', [m n]);
    
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
    data.input.tf = affine3d;

    data.input.scan = pcread(scan_filename);
    data.input.tf.T = tf_lidar2map.T * tf_map2camera.T;
    data.input.image = imread(image_filename);

    
    % TODO: get from file saved in ros!!!
    intrinsic_matrix = ...
        [461.05267333984375      0.0                              318.204833984375; ...   
         0.0                                    461.2838134765625  186.91806030273438; ...
         0.0                                    0.0                              1.0];
     [m, n, ~] = size(data.input.image);
    params.camera_params = cameraParameters(...
                                                 'IntrinsicMatrix', intrinsic_matrix', ...
                                                 'ImageSize', [m n]);

    % lidar parameters
    params.lidar_model = params.id_vlp16;
    params.lidar_parameters = [];
    params.lidar_parameters = fillVlp16PuckCfg(data.input.scan);
end

end

