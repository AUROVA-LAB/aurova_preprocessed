function [max_elevation, min_elevation, max_azimuth, min_azimuth] = maxMinElevation(scan)

max_azimuth = -10000.0;
min_azimuth = 10000.0;
max_elevation = -10000.0;
min_elevation = 10000.0;
   
for n = 1:scan.Count
    x = scan.Location(n, 1);
    y = scan.Location(n, 2);
    z = scan.Location(n, 3);
    
    [~, azimuth, elevation] = cartesian2SphericalInDegrees(x, y, z);
    
    if (elevation > max_elevation)
         max_elevation = elevation;
    end
     if (elevation < min_elevation)
         min_elevation = elevation;
     end
     
     if (azimuth > max_azimuth)
         max_azimuth = azimuth;
    end
     if (azimuth < min_azimuth)
         min_azimuth = azimuth;
     end
end

