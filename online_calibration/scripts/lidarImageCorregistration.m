function plot_info = lidarImageCorregistration(data_prep, params)

% select pair's and roi's from discontinuities
descriptor = selecLidarKeyPoints(data_prep, params);

% calculation corregister asociation in pixels
plot_info = findImageKpCorrespondence(data_prep, descriptor, params);

end