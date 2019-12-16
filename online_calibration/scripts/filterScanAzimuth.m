function scan_filtered = filterScanAzimuth(scan, lidar_config)

for n = 1:scan.Count
    
    point = scan.Location(n, :);
    [v, u] = point2SphericalGrid(point, lidar_config);
    
    scan_filtered(v, u) =point (3);
end

