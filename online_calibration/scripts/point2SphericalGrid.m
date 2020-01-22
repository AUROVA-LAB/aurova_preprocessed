function [v, u] = point2SphericalGrid(point, lidar_config)

INVALID_VALUE = -1;
v =  INVALID_VALUE;
u =  INVALID_VALUE;

[range, azimuth, elevation] = cartesian2SphericalInDegrees(point(1), point(2), point(3));

if (azimuth <= lidar_config.max_azimuth && azimuth >= lidar_config.min_azimuth ...
  && elevation <= lidar_config.max_elevation && elevation >= lidar_config.min_elevation)
    u = round((azimuth - lidar_config.min_azimuth) / lidar_config.res_azimuth) + 1;
    v = round((elevation - lidar_config.min_elevation) / lidar_config.res_elevation) + 1;
    if (u < 1 || u > lidar_config.azimuth_cells || v < 1 || v > lidar_config.elevation_cells)
      v =  INVALID_VALUE;
      u =  INVALID_VALUE;
    end
end

end

