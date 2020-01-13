function scan_filtered = filterScanAzimuth(scan, is_kitti, threshold_dw, threshold_up, base_pixel)

scan_filtered = scan;

if is_kitti
    for i = 2:scan.Count-1
        
        lidar_config = [];
        lidar_config.res_azimuth = 0.18;
        lidar_config.max_range = 100.0;
        
        % actual point
        [range_act, azimuth_act, elevation_act] = cartesian2SphericalInDegrees(scan.Location(i, 1), scan.Location(i, 2), scan.Location(i, 3));
        
        % previous point
         [range_pre, azimuth_pre, elevation_pre] = cartesian2SphericalInDegrees(scan.Location(i-1, 1), scan.Location(i-1, 2), scan.Location(i-1, 3));
         if (abs(azimuth_act - azimuth_pre) > lidar_config.res_azimuth * 4)
            range_pre = lidar_config.max_range;
         end
        
         % posterior point
         [range_pos, azimuth_pos, elevation_pos] = cartesian2SphericalInDegrees(scan.Location(i+1, 1), scan.Location(i+1, 2), scan.Location(i+1, 3));
         if (abs(azimuth_act - azimuth_pos) > lidar_config.res_azimuth * 4)
            range_pos = lidar_config.max_range;
         end
        
         % filter point
        mask = [range_pre-range_act, 0.0, range_pos-range_act];
        max_dif = max(mask);
        if max_dif >= threshold_dw
            scope = max_dif - threshold_dw;
            scope_max = threshold_up - threshold_dw;
            if (scope > scope_max)
              scope = scope_max;
            end
            scan_filtered.Intensity(i) = scope / scope_max * base_pixel;
        end
    end
    
else

    lidar_config = fillLidarCfg(scan);
    
    num_slices = lidar_config.elevation_cells;
    scan_slices = cell(num_slices, 1);
    index_slices = cell(num_slices, 1);

    % structure scan data in slices
    for n = 1:scan.Count

        point = scan.Location(n, :);
        [v, u] = point2SphericalGrid(point, lidar_config);

        if v > -1
            scan_slices{v} = cat(1, scan_slices{v}, point);
            index_slices{v} = cat(1, index_slices{v}, n);
            %scan_filtered(v, u) =point (3);
        end
    end

    % filter slices of scans
    for i = 1:num_slices
        [len, d] = size(scan_slices{i});
        points = scan_slices{i};
        index = index_slices{i};

        for j = 1:len 

            % actual point
            [range_act, azimuth_act, el] = cartesian2SphericalInDegrees(points(j, 1), points(j, 2), points(j, 3));

            % previous point
            if j - 1 < 1
                range_pre = range_act;
            else
                [range_pre, azimuth_pre, el] = cartesian2SphericalInDegrees(points(j-1, 1), points(j-1, 2), points(j-1, 3));
                if (abs(azimuth_act - azimuth_pre) > lidar_config.res_azimuth * 4)
                    range_pre = lidar_config.max_range;
                end
            end

            % porterior point
            if j + 1 > len
                range_pos = range_act;
            else
                [range_pos, azimuth_pos, el] = cartesian2SphericalInDegrees(points(j+1, 1), points(j+1, 2), points(j+1, 3));
                if (abs(azimuth_act - azimuth_pos) > lidar_config.res_azimuth * 4)
                    range_pos = lidar_config.max_range;
                end
            end

            % filter point
            mask = [range_pre-range_act, 0.0, range_pos-range_act];
            max_dif = max(mask);
            if max_dif >= threshold_dw
                scope = max_dif - threshold_dw;
                scope_max = threshold_up - threshold_dw;
                if (scope > scope_max)
                  scope = scope_max;
                end
                n = index(j);
                scan_filtered.Intensity(n) = scope / scope_max * base_pixel;
            end

    %         % debug
    %         [v, u] = point2SphericalGrid(points(j, :), lidar_config);
    %         scan_image(v, u) = mapDistanceToUint(points (j, 3), 10.0, 255);
    %         scan_edges(v, u) = scope / scope_max * base_pixel;
        end
    end
end

end

