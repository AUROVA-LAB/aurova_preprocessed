function analizeResults (results, data)

max_inliers = data.matches_acum.max;
groundt_loc = data.tf_miss.T(4, 1:3);
[gt_y, gt_p, gt_r] = dcm2angle(data.tf_miss.T(1:3, 1:3));
[~, num_samples] = size(results);

inliers_hist (1:max_inliers) = 0;
inliers_err_loc (1:max_inliers) = 0;
inliers_err_ori (1:max_inliers) = 0;

for i = 1:num_samples
    index = sum(results{i}.inliers);
    error_xyz = results{i}.location - groundt_loc;
    [err_y, err_p, err_r] = dcm2angle(results{i}.orientation);
    [error_loc, ~, ~] = cartesian2SphericalInDegrees(error_xyz(1), error_xyz(2), error_xyz(3));
    [error_ori, ~, ~] = cartesian2SphericalInDegrees(err_y - gt_y, err_p - gt_p, err_r - gt_r);
    inliers_err_loc(index) = inliers_err_loc(index) + error_loc;
    inliers_err_ori(index) = inliers_err_ori(index) + error_ori;
    inliers_hist(index) = inliers_hist(index) + 1;
end

figure
bar(inliers_hist(1:max_inliers/2))
figure
bar(inliers_err_ori(1:max_inliers/2) ./ inliers_hist(1:max_inliers/2))
figure
bar(inliers_err_loc(1:max_inliers/2) ./ inliers_hist(1:max_inliers/2))

end

