function noise_model = noiseModel (obs, data)

max_inliers = data.matches_acum.max;
groundt_loc = data.tf_err.T(4, 1:3);
[gt_y, gt_p, gt_r] = dcm2angle(data.tf_err.T(1:3, 1:3));
[~, num_samples] = size(obs);

inliers_hist (1:max_inliers) = 0;
inliers_err_loc (1:max_inliers) = 0;
inliers_err_ori (1:max_inliers) = 0;
tdata = 1:max_inliers;

for i = 1:num_samples
    index = sum(obs{i}.inliers);
    error_xyz = obs{i}.location - groundt_loc;
    [err_y, err_p, err_r] = dcm2angle(obs{i}.orientation);
    [error_loc, ~, ~] = cartesian2SphericalInDegrees(error_xyz(1), error_xyz(2), error_xyz(3));
    [error_ori, ~, ~] = cartesian2SphericalInDegrees(err_y - gt_y, err_p - gt_p, err_r - gt_r);
    inliers_err_loc(index) = inliers_err_loc(index) + error_loc;
    inliers_err_ori(index) = inliers_err_ori(index) + error_ori;
    inliers_hist(index) = inliers_hist(index) + 1;
end

inliers_err_ori = (inliers_err_ori ./ inliers_hist) * (180/pi);
inliers_err_loc = inliers_err_loc ./ inliers_hist;

% fit exponential function
A = 0.34;
lambda = 0.034;
noise_model_loc = A*exp(-lambda*tdata);
A = 0.86;
lambda = 0.031;
noise_model_ori = A*exp(-lambda*tdata);

noise_model = [];
noise_model.loc = noise_model_loc;
noise_model.ori = noise_model_ori;

figure
bar(inliers_hist(1:max_inliers))
figure
bar(inliers_err_ori(1:max_inliers))
hold on
plot(noise_model_ori)
figure
bar(inliers_err_loc(1:max_inliers))
hold on
plot(noise_model_loc)

end

