function [data, data_prep, matches] = extTargetlessCalibration(data, params)

% PART I
data_prep = preprocessData(data, params);

% PART II
matches = lidarImageCorregistration(data_prep, params);

% PART III
data = errorCorrection (data, matches, params);

end