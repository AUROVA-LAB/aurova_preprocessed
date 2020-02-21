clear, clc, close all

disp('*** init program: load parameters (only Matlab functions) ***');
[data, params, experiments] = getConfigurationParams();

M = experiments.num_datasets;
results = {};

t = tic;
for j = 5
    N = experiments.num_samples(j);
    experiments.tf_miss = generateMisscalibration();
    for i = 1:N       
        experiments.id_dataset = j;
        experiments.id_sample = i;  
        
        disp([j i])
        disp('*** reading data (only Matlab functions) ***')
        [data, params] = readFileData(data, params, experiments);

        disp('*** init extrinsic targetless calibration (library) ***')
        [data, data_prep, matches] = extTargetlessCalibration(data, params);
        
        disp('*** plot results (only Matlab functions) ***')
        plotResults(data, data_prep, matches, params, experiments);
        
        if data.matches_acum.num >= data.matches_acum.max
            results = [results data.output];
        end
    end
end
disp(toc(t))