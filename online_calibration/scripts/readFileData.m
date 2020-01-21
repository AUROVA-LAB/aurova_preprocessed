function [image, scan, tf] = readFileData(config)

if config.is_kitti
    
    % selection of camera in kitti
    camera = config.kitti_cam_id;
    base_dir = config.base_dir_cell{config.id_dataset};
    calib_dir = config.calib_dir_cell{config.id_dataset};
    index = config.id_sample;

    % load kitti image
    image = imread(sprintf('%s/image_%02d/data/%010d.png', base_dir, camera, index));

    % load kitti velodyne points
    fid = fopen(sprintf('%s/velodyne_points/data/%010d.bin', base_dir, index), 'rb');
    velo = fread(fid, [4 inf], 'single')';
    fclose(fid);
    scan_lidarframe = pointCloud(velo(:, 1:3));
    [n, d] = size(velo);
    intensity(1:n) = 0;
    scan_lidarframe.Intensity = intensity';

    % load calibration, and compute projection matrix velodyne->image plane
    tf_lidar2cam = affine3d;
    calib = loadCalibrationCamToCam(fullfile(calib_dir,'calib_cam_to_cam.txt'));
    tf_lidar2cam.T = loadCalibrationRigid(fullfile(calib_dir,'calib_velo_to_cam.txt'))';
    R_cam_to_rect = eye(4);
    R_cam_to_rect(1:3,1:3) = calib.R_rect{1};
    K_rect = calib.P_rect{camera+1} * R_cam_to_rect;

    camera_params = [];
    [m, n, c] = size(image); 
    camera_params.image_size_ = [m n];
    camera_params.intrinsic_matrix_ = K_rect ; %calib.K{camera+1}
else
end

end

