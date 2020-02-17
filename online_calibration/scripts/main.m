clear, clc, close all

disp('*** init program: load parameters (only Matlab functions) ***');
[params, experiments] = getConfigurationParams();

M = experiments.num_datasets;

t = tic;
for j = [5 8 9]
    N = experiments.num_samples(j);
    experiments.tf_miss = generateMisscalibration();
    for i = 10       
        experiments.id_dataset = j;
        experiments.id_sample = i;  
        
        disp('*** reading data (only Matlab functions) ***')
        [data, params] = readFileData(params, experiments);

        disp('*** init extrinsic targetless calibration (library) ***')
        [data, data_prep, matches] = extTargetlessCalibration(data, params);
        
        disp('*** plot results (only Matlab functions) ***')
        plotResults(data, data_prep, matches, params, experiments);
    end
end
disp(toc(t))