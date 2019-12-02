clear, clc, close all

%*************** read files *****************%
index = 150;
window_size = 50;
scan_filename_base = 'images/input/raw_data_01/scan';
scan_filename = strcat(scan_filename_base, num2str(index,'%d.pcd'));
scan = pcread(scan_filename);

cloud = scan;
index = index + 1;
step = 1;
previous_tform = pcregistericp(scan, cloud);

for n = index:step:index + window_size
    
    scan_filename = strcat(scan_filename_base, num2str(n,'%d.pcd'));
    scan = pcread(scan_filename);
    scan = helperProcessPointCloud(scan);
    scan = pctransform(scan, previous_tform);
        
    transform = pcregistericp(scan, cloud);
    scan = pctransform(scan, transform);
    
    cloud = pcmerge(cloud, scan, 0.1);
    previous_tform = transform;
end

pcshow(cloud, 'MarkerSize', 30);