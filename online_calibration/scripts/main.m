clear, clc, close all

disp('*** reading data (only Matlab functions) ***')
[params, experiments] = getConfigurationParams();
[data, params] = readFileData(params, experiments);

disp('*** init extrinsic targetless calibration ***')
[tf, data_prep, plot_info] = extTargetlessCalibration(data, params, experiments); %TODO: remove experiments variable

disp('*** plot results ***')
plotResults(data_prep, plot_info, params);