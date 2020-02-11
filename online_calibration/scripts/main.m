clear, clc, close all

disp('*** init program: load parameters ***');
[params, experiments] = getConfigurationParams();

M = experiments.id_dataset;
N = experiments.id_sample;
results = [];

t = tic;
for j = M:M
    for i = 1:N        
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
disp(toc(t))