function camera_params = getCameraParams()

%**** parameter of Realsense D435 camera ***%
%***** TODO: get from file saved in ros!!! *****%

image_size = [640 360];
intrinsic_matrix = ...
    [461.05267333984375      0.0                              318.204833984375; ... 
     0.0                                    461.2838134765625  186.91806030273438; ...
     0.0                                    0.0                              1.0];
 
 camera_params = cameraParameters('IntrinsicMatrix', intrinsic_matrix, 'ImageSize', image_size);

end

