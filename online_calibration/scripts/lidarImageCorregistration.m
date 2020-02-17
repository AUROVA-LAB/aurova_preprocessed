function matches = lidarImageCorregistration(data_prep, params)

% select pair's and roi's from discontinuities
descriptors = selecLidarKeyPoints(data_prep, params);

% calculation corregister asociation in pixels
matches = findImageKpCorrespondence(data_prep, descriptors, params);

end