function [data, params, experiments] = getConfigurationParams()

params.sigma = 20;
params.base = 255;

params.threshold_dw = 0.5;
params.threshold_up = 10;
params.threshold_dsc = 0;
params.threshold_sbl = 0;

params.dist_factor = 0.8;
params.rot_max = 30;
params.area = 20;
params.distance_w = 200;
params.max_distance = 40.0;
params.min_intensity = 200;
params.k = 10; %always pair number

params.lidar_model = 0; %fill in read data phase
params.id_vlp16 = 16;
params.id_kitti64 = 64;

data = [];
data.matches_acum = [];
data.matches_acum.max = 200;
data.matches_acum.num = 0;
data.matches_acum.kp_src =[];
data.matches_acum.kp_tmp =[];
data.matches_acum.wp_tmp =[];

experiments.id_dataset = 1;
experiments.id_sample = 1;
experiments.kitti_cam_id = 0;
experiments.fileout = 'results/20200221/frame';
experiments.num_datasets = 10;
experiments.is_kitti(1) = true;
experiments.num_samples(1) = 185;
experiments.calib_dir_cell{1} = 'raw_data/2011_09_28'; 
experiments.base_dir_cell{1} = 'raw_data/2011_09_28/2011_09_28_drive_0016_sync'; % campus
experiments.is_kitti(2) = true;
experiments.num_samples(2) = 88;
experiments.calib_dir_cell{2} = 'raw_data/2011_09_28'; 
experiments.base_dir_cell{2} = 'raw_data/2011_09_28/2011_09_28_drive_0037_sync';% campus
experiments.is_kitti(3) = true;
experiments.num_samples(3) = 144;
experiments.calib_dir_cell{3} = 'raw_data/2011_09_28'; 
experiments.base_dir_cell{3} = 'raw_data/2011_09_28/2011_09_28_drive_0043_sync'; % campus
experiments.is_kitti(4) = true;
experiments.num_samples(4) = 296;
experiments.calib_dir_cell{4} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{4} = 'raw_data/2011_09_26/2011_09_26_drive_0015_sync';% road
experiments.is_kitti(5) = true;
experiments.num_samples(5) = 77;
experiments.calib_dir_cell{5} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{5} = 'raw_data/2011_09_26/2011_09_26_drive_0052_sync';% road
experiments.is_kitti(6) = true;
experiments.num_samples(6) = 130;
experiments.calib_dir_cell{6} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{6} = 'raw_data/2011_09_26/2011_09_26_drive_0035_sync'; % residential
experiments.is_kitti(7) = true;
experiments.num_samples(7) = 124;
experiments.calib_dir_cell{7} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{7} = 'raw_data/2011_09_26/2011_09_26_drive_0046_sync'; % residential
experiments.is_kitti(8) = true;
experiments.num_samples(8) = 153;
experiments.calib_dir_cell{8} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{8} = 'raw_data/2011_09_26/2011_09_26_drive_0005_sync';% city
experiments.is_kitti(9) = true;
experiments.num_samples(9) = 269;
experiments.calib_dir_cell{9} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{9} = 'raw_data/2011_09_26/2011_09_26_drive_0018_sync'; % city
experiments.is_kitti(10) = true;
experiments.num_samples(10) = 226;
experiments.calib_dir_cell{10} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{10} = 'raw_data/2011_09_26/2011_09_26_drive_0106_sync';% city


end

