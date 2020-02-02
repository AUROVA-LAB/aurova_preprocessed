function [tf, data_prep, plot_info] = extTargetlessCalibration(data, params)

data_prep = prerpocessData(data, params);

plot_info = lidarImageCorregistration(data_prep, params);

% tf = errorMinimization (tf, errors);

% bypass for compilation
tf = data_prep;

end