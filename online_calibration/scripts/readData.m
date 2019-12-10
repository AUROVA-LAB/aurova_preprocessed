function [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera, image] = ...
    readData(read_mat, index, window_size)

if read_mat
    scans_lidarframe = load(num2str(index,'mat_data/scans_lidarframe%d.mat'), 'scans_lidarframe');
    scans_mapframe = load(num2str(index,'mat_data/scans_mapframe%d.mat'), 'scans_mapframe');
    tfs_lidar2map = load(num2str(index,'mat_data/tfs_lidar2map%d.mat'), 'tfs_lidar2map');
    tfs_map2camera = load(num2str(index,'mat_data/tfs_map2camera%d.mat'), 'tfs_map2camera');
    image = load(num2str(index,'mat_data/image%d.mat'), 'image');
    scans_lidarframe = scans_lidarframe.scans_lidarframe;
    scans_mapframe = scans_mapframe.scans_mapframe;
    tfs_lidar2map = tfs_lidar2map.tfs_lidar2map;
    tfs_map2camera = tfs_map2camera.tfs_map2camera;
    image = image.image;
else
    scan_filename_base = 'raw_data/sec_0101/scan';
    tf_filename_base = 'raw_data/sec_0101/tf';
    image_filename_base = 'raw_data/sec_0101/image';
    image_filename = strcat(image_filename_base, num2str(index,'%d.jpg'));
    image = imread(image_filename);
    step = -1;
    ini = index + window_size/2;
    endl = index - window_size/2;
    if endl < 0
        endl = 0;
    end
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
    save(num2str(index,'mat_data/scans_lidarframe%d.mat'), 'scans_lidarframe');
    save(num2str(index,'mat_data/scans_mapframe%d.mat'), 'scans_mapframe');
    save(num2str(index,'mat_data/tfs_lidar2map%d.mat'), 'tfs_lidar2map');
    save(num2str(index,'mat_data/tfs_map2camera%d.mat'), 'tfs_map2camera');
    save(num2str(index,'mat_data/image%d.mat'), 'image');
end

end

