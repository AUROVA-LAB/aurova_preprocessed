function data = lidarImageCorregistration(data, params)

% select pair's and roi's from discontinuities
descriptors = selecLidarKeyPoints(data.process, params);

% calculation corregister asociation in pixels
matches = findImageKpCorrespondence(data.process, descriptors, params);
data.matches.current = matches;

end