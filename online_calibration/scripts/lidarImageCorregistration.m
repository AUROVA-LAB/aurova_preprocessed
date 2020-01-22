function plot_info = lidarImageCorregistration(data_prep, params, experiments)

% select pair's and roi's from discontinuities
descriptor = selecLidarKpManually(experiments);
%descriptor = selecLidarKeyPoints(experiments);

% calculation corregister asociation in pixels
plot_info = findImageKpCorrespondence(data_prep, descriptor, params);

end