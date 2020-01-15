function image = gpuProofsRead()

% selection of camera in kitti
camera = 2;
index = 70;

% load kitti image
base_dir = 'raw_data/2011_09_26/2011_09_26_drive_0005_sync';
image = imread(sprintf('%s/image_%02d/data/%010d.png', base_dir, camera, index));


end