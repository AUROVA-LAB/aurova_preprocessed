clear, clc, close all

disp('*** init program: load parameters ***');
[params, experiments] = getConfigurationParams();

M = experiments.id_dataset;
N = experiments.id_sample;
results = [];
results.error_3 = 0;
results.error_5 = 0;
results.error_8 = 0;
results.error_15 = 0;
results.error_x = 0;
results.error_signal = [];

for j = 1:M
    for i = N/2:N
        
        experiments.id_dataset = j;
        experiments.id_sample = i;
        
        disp('*** reading data (only Matlab functions) ***')
        [data, params] = readFileData(params, experiments);

        disp('*** init extrinsic targetless calibration ***')
        [tf, data_prep, plot_info] = extTargetlessCalibration(data, params); %TODO: remove experiments variable

        disp('*** plot results ***')
        results = plotResults(data, data_prep, plot_info, params, experiments, results);
    end
end