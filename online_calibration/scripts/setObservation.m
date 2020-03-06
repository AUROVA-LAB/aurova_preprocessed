function observation = setObservation(ext_obs, noise_model)

observation = [];

[yaw, pitch, roll] = dcm2angle(ext_obs.orientation);

observation.obs = [ext_obs.location yaw*(180/pi) pitch*(180/pi) roll*(180/pi)]';
observation.cov = eye(length(observation.obs));

inliers = sum(ext_obs.inliers);

observation.cov(1, 1) = noise_model.loc(inliers);
observation.cov(2, 2) = noise_model.loc(inliers);
observation.cov(3, 3) = noise_model.loc(inliers);
observation.cov(4, 4) = noise_model.ori(inliers);
observation.cov(5, 5) = noise_model.ori(inliers);
observation.cov(6, 6) = noise_model.ori(inliers);

end