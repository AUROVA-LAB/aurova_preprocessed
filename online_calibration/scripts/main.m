clear, clc, close all

disp('*** reading data (only Matlab functions) ***')
[params, experiments] = getConfigurationParams();
[data, params] = readFileData(experiments, params);

disp('*** init extrinsic targetless calibration ***')
tf = extTargetlessCalibration(data, params);
