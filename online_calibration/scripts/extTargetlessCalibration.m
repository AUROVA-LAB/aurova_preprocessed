function [data, data_prep, matches] = extTargetlessCalibration(data, params)

% PART I
data_prep = preprocessData(data, params);

% PART II
matches = lidarImageCorregistration(data_prep, params);
data = acumMatches(data, data_prep, matches);

% PART III
data = errorCorrection (data, params);

end