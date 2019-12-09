function [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera] = ...
    readData(is_m_data, index, window_size)

if is_m_data
    scans_lidarframe = load('mat_data/scans_lidarframe.mat', 'scans_lidarframe');
    scans_mapframe = load('mat_data/scans_mapframe.mat', 'scans_mapframe');
    tfs_lidar2map = load('mat_data/tfs_lidar2map.mat', 'tfs_lidar2map');
    tfs_map2camera = load('mat_data/tfs_map2camera.mat', 'tfs_map2camera');
    scans_lidarframe = scans_lidarframe.scans_lidarframe;
    scans_mapframe = scans_mapframe.scans_mapframe;
    tfs_lidar2map = tfs_lidar2map.tfs_lidar2map;
    tfs_map2camera = tfs_map2camera.tfs_map2camera;
else
    scan_filename_base = 'raw_data/input/raw_data_01/scan';
    tf_filename_base = 'raw_data/input/raw_data_01/tf';
    step = -1;
    ini = index + window_size/2;
    endl = index - window_size/2;
    scans_lidarframe = {};
    scans_mapframe = {};
    tfs_lidar2map = {};
    tfs_map2camera = {};
    for n = ini:step:endl
        scan_filename = strcat(scan_filename_base, num2str(n,'%d.pcd'));
        tf_filename = strcat(tf_filename_base, num2str(n,'%d.csv'));
        scan = pcread(scan_filename);
        xyz_rpy = csvread(tf_filename);
        tform_lidar2map = getTfAffineMatrix(xyz_rpy, 1);
        tform_map2camera = getTfAffineMatrix(xyz_rpy, 2);
        scan = helperProcessPointCloud(scan);
        scan_mapframe = pctransform(scan, tform_lidar2map);
        tfs_lidar2map = [tfs_lidar2map; {tform_lidar2map}];
        tfs_map2camera = [tfs_map2camera; {tform_map2camera}];
        scans_lidarframe = [scans_lidarframe; {scan}];
        scans_mapframe = [scans_mapframe; {scan_mapframe}];
        
    end
    save('mat_data/scans_lidarframe.mat', 'scans_lidarframe');
    save('mat_data/scans_mapframe.mat', 'scans_mapframe');
    save('mat_data/tfs_lidar2map.mat', 'tfs_lidar2map');
    save('mat_data/tfs_map2camera.mat', 'tfs_map2camera');
end

end

