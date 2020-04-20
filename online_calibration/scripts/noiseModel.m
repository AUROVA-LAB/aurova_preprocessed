function noise_model = noiseModel (obs, gt, resolution)

groundt_loc = gt(1:3);
gt_y = gt(4);
gt_p = gt(5);
gt_r = gt(6);
[~, num_samples] = size(obs);

inliers_hist (1:resolution) = 0;
inliers_err_loc (1:resolution) = 0;
inliers_err_ori (1:resolution) = 0;
tdata = 1:resolution;

for i = 1:num_samples
    num_in = sum(obs{i}.inliers);
    num_tot = length(obs{i}.inliers);
    index = round((num_in / num_tot) * resolution);
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
limit_plot = 100;

figure
bar(inliers_hist(1:limit_plot))
grid
subplot(1, 2, 1)
bar(inliers_err_ori(1:limit_plot))
axis([0 100 0 1])
xlabel('Inliers (%)')
ylabel('mean error (degrees)')
hold on
plot(noise_model_ori(1:limit_plot), 'r', 'LineWidth', 2)
pbaspect([1.5 1 1])
grid
subplot(1, 2, 2)
bar(inliers_err_loc(1:limit_plot))
axis([0 100 0 0.5])
xlabel('Inliers (%)')
ylabel('mean error (meters)')
hold on
plot(noise_model_loc(1:limit_plot), 'r', 'LineWidth', 2)
pbaspect([1.5 1 1])
grid

end

