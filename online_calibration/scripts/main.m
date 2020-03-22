clear, clc, close all

disp('*** init program: load parameters (only Matlab functions) ***');
[data, params, experiments] = getConfigurationParams();

M = experiments.num_datasets;
ext_obs = {};
ext_stt = {};
state = initState();
[data.input.tf_err, gt] = generateMisscalibration();
load('noise_model.mat')

% datasets test
ind = 2:M;
ind = fliplr(ind);

t = tic;
for j = 9
    N = experiments.num_samples(j);
    for i = 1 %1:N       
        experiments.id_dataset = j;
        experiments.id_sample = i;  
        
        disp([j i])
        disp('*** reading data (only Matlab functions) ***')
        [data, params] = readFileData(data, params, experiments);

        disp('*** init extrinsic targetless calibration (library) ***')
        data = extTargetlessCalibration(data, params);
        
        disp('*** plot results (only Matlab functions) ***')
        plotResults(data, params, experiments);
        
        if data.matches.num >= data.matches.max
            ext_obs = [ext_obs data.output];
            observation = setObservation(data.output, noise_model);
            state = updateState(observation, state);
            ext_stt = [ext_stt state];
        end
    end
end
disp(toc(t))