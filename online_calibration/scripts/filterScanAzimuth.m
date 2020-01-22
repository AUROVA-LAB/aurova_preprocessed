function scan_filtered = filterScanAzimuth(scan, params)

scan_filtered = scan;

if params.lidar_model == params.id_kitti64
    for i = 2:scan.Count-1
        
        % actual point
        [range_act, azimuth_act, ~] = cartesian2SphericalInDegrees(scan.Location(i, 1), scan.Location(i, 2), scan.Location(i, 3));
        
        % previous point
         [range_pre, azimuth_pre, ~] = cartesian2SphericalInDegrees(scan.Location(i-1, 1), scan.Location(i-1, 2), scan.Location(i-1, 3));
         if (abs(azimuth_act - azimuth_pre) > params.lidar_parameters.res_azimuth * 4)
            range_pre = params.lidar_parameters.max_range;
         end
        
         % posterior point
         [range_pos, azimuth_pos, ~] = cartesian2SphericalInDegrees(scan.Location(i+1, 1), scan.Location(i+1, 2), scan.Location(i+1, 3));
         if (abs(azimuth_act - azimuth_pos) > params.lidar_parameters.res_azimuth * 4)
            range_pos = params.lidar_parameters.max_range;
         end
        
         % filter point
        mask = [range_pre-range_act, 0.0, range_pos-range_act];
        max_dif = max(mask);
        if max_dif >= params.threshold_dw
            scope = max_dif - params.threshold_dw;
            scope_max = params.threshold_up - params.threshold_dw;
            if (scope > scope_max)
              scope = scope_max;
            end
            scan_filtered.Intensity(i) = scope / scope_max * params.base;
        end
    end
    
elseif params.lidar_model == params.id_vlp16
    
    num_slices = params.lidar_parameters.elevation_cells;
    scan_slices = cell(num_slices, 1);
    index_slices = cell(num_slices, 1);

    % structure scan data in slices
    for n = 1:scan.Count

        point = scan.Location(n, :);
        [v, ~] = point2SphericalGrid(point, params.lidar_parameters);

        if v > -1
            scan_slices{v} = cat(1, scan_slices{v}, point);
            index_slices{v} = cat(1, index_slices{v}, n);
        end
    end

    % filter slices of scans
    for i = 1:num_slices
        [len, ~] = size(scan_slices{i});
        points = scan_slices{i};
        index = index_slices{i};

        for j = 1:len 

            % actual point
            [range_act, azimuth_act, ~] = cartesian2SphericalInDegrees(points(j, 1), points(j, 2), points(j, 3));

            % previous point
            if j - 1 < 1
                range_pre = range_act;
            else
                [range_pre, azimuth_pre, ~] = cartesian2SphericalInDegrees(points(j-1, 1), points(j-1, 2), points(j-1, 3));
                if (abs(azimuth_act - azimuth_pre) > params.lidar_parameters.res_azimuth * 4)
                    range_pre = params.lidar_parameters.max_range;
                end
            end

            % porterior point
            if j + 1 > len
                range_pos = range_act;
            else
                [range_pos, azimuth_pos, ~] = cartesian2SphericalInDegrees(points(j+1, 1), points(j+1, 2), points(j+1, 3));
                if (abs(azimuth_act - azimuth_pos) > params.lidar_parameters.res_azimuth * 4)
                    range_pos = params.lidar_parameters.max_range;
                end
            end

            % filter point
            mask = [range_pre-range_act, 0.0, range_pos-range_act];
            max_dif = max(mask);
            if max_dif >= params.threshold_dw
                scope = max_dif - params.threshold_dw;
                scope_max = params.threshold_up - params.threshold_dw;
                if (scope > scope_max)
                  scope = scope_max;
                end
                n = index(j);
                scan_filtered.Intensity(n) = scope / scope_max * params.base;
            end
        end
    end
end

end

