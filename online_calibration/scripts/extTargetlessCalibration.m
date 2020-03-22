function data = extTargetlessCalibration(data, params)

% PART I
data = preprocessData(data, params);

% PART II
data = lidarImageCorregistration(data, params);
data = acumMatches(data);

% PART III
data = errorCorrection (data, params);

end