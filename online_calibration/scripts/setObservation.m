function observation = setObservation(ext_obs, noise_model)

observation = [];

[yaw, pitch, roll] = dcm2angle(ext_obs.orientation);

observation.obs = [ext_obs.location yaw*(180/pi) pitch*(180/pi) roll*(180/pi)]';
observation.cov = eye(length(observation.obs));

resolution = 200;
num_in = sum(ext_obs.inliers);
num_tot = length(ext_obs.inliers);
index = round((num_in / num_tot) * resolution);

observation.cov(1, 1) = noise_model.loc(index);
observation.cov(2, 2) = noise_model.loc(index);
observation.cov(3, 3) = noise_model.loc(index);
observation.cov(4, 4) = noise_model.ori(index);
observation.cov(5, 5) = noise_model.ori(index);
observation.cov(6, 6) = noise_model.ori(index);

end