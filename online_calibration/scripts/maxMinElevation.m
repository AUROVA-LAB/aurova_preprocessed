function [max_elevation, min_elevation] = maxMinElevation(scan)

max_elevation = -10000.0;
min_elevation = 10000.0;
   
for n = 1:scan.Count
    x = scan.Location(n, 1);
    y = scan.Location(n, 2);
    z = scan.Location(n, 3);
    
    [range, azimuth, elevation] = cartesian2SphericalInDegrees(x, y, z);
    
    if (elevation > max_elevation)
         max_elevation = elevation;
    end
     if (elevation < min_elevation)
         min_elevation = elevation;
     end
end

