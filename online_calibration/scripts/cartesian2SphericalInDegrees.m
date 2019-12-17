function [range, azimuth, elevation] = cartesian2SphericalInDegrees(x, y, z)

  range = sqrt((x * x) + (y * y) + (z * z));

  azimuth = atan2(y, x) * 180.0 / pi;
  elevation = atan2(sqrt((x * x) + (y * y)), z) * 180.0 / pi;

%   if (azimuth < 0)
%     azimuth = azimuth + 360.0;
%   end
%   if (azimuth >= 360)
%     azimuth = azimuth - 360;
%   end
% 
%   if (elevation < 0)
%     elevation = elevation + 360.0;
%   end
%   if (elevation >= 360)
%     elevation = elevation - 360;
%   end
  
end

