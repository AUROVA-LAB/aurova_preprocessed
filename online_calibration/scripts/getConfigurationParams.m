function config = getConfigurationParams()

config.is_kitti = true;
config.id_dataset = 2;
config.id_sample = 70;
config.id_pair = 1;
config.kitti_cam_id = 2;
config.sigma = 20;
config.sigma_flt = 5;
config.sobel_fact = 0.5;
config.base = 255;
config.threshold_dw = 0.5;
config.threshold_up = 20;
config.exec_flag = [true true true true true];
config.filenames_cell{1} = 'raw_data/sec_0101/'; % Aurova paths
config.filenames_cell{2} = 'raw_data/sec_0105/';
config.filenames_cell{3} = 'raw_data/sec_0107/';
config.filenames_cell{4} = 'raw_data/sec_0202/';
config.filenames_cell{5} = 'raw_data/sec_0203/';
config.calib_dir_cell{1} = 'raw_data/2011_09_26'; % Kitti paths
config.base_dir_cell{1} = 'raw_data/2011_09_26/2011_09_26_drive_0018_sync';
config.calib_dir_cell{2} = 'raw_data/2011_09_26'; 
config.base_dir_cell{2} = 'raw_data/2011_09_26/2011_09_26_drive_0005_sync';

end

