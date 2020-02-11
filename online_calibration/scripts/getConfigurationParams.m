function [params, experiments] = getConfigurationParams()

params.sigma = 20;
params.base = 255;

params.threshold_dw = 0.5;
params.threshold_up = 10;
params.threshold_dsc = 0;
params.threshold_sbl = 0;

params.dist_factor = 0.8;
params.rot_max = 30;
params.area = 100;
params.distance_w = 200;
params.max_distance = 40.0;
params.min_intensity = 200;
params.k = 10; %always pair number

params.lidar_model = 0; %fill in read data phase
params.id_vlp16 = 16;
params.id_kitti64 = 64;

experiments.is_kitti = true;
experiments.id_dataset = 1;
experiments.id_sample = 160;
experiments.id_pair = 1;
experiments.kitti_cam_id = 2;
experiments.fileout = 'results/20200211/frame';
experiments.filenames_cell{1} = 'raw_data/sec_0101/'; % Aurova paths
experiments.filenames_cell{2} = 'raw_data/sec_0105/';
experiments.filenames_cell{3} = 'raw_data/sec_0107/';
experiments.filenames_cell{4} = 'raw_data/sec_0202/';
experiments.filenames_cell{5} = 'raw_data/sec_0203/';
experiments.calib_dir_cell{1} = 'raw_data/2011_09_26'; % Kitti paths
experiments.base_dir_cell{1} = 'raw_data/2011_09_26/2011_09_26_drive_0018_sync';
experiments.calib_dir_cell{2} = 'raw_data/2011_09_26'; 
experiments.base_dir_cell{2} = 'raw_data/2011_09_26/2011_09_26_drive_0005_sync';

end

