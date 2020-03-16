clear, clc, close all

disp('*** init program: load parameters (only Matlab functions) ***');
[data, params, experiments] = getConfigurationParams();

M = experiments.num_datasets;
ext_obs = {};
ext_stt = {};
state = initState();
[data.tf_err, gt] = generateMisscalibration();
load('noise_model.mat')

% datasets test
ind = 1:M;
ind = cat(2, ind, ind);
ind = cat(2, ind, [9 9 9]);
ind = cat(2, ind, [10 10 10 10]);
%ind = fliplr(ind);

t = tic;
for j = ind
    N = experiments.num_samples(j);
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
            ext_obs = [ext_obs data.output];
            observation = setObservation(data.output, noise_model);
            state = updateState(observation, state);
            ext_stt = [ext_stt state];
        end
    end
end
disp(toc(t))