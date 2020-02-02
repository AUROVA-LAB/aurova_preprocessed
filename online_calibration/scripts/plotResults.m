function results = plotResults(data_prep, plot_info, params, experiments, results)

sobel_nrm = data_prep.img_sobel / max(max(data_prep.img_sobel));
sobel_nrm = uint8(sobel_nrm * params.base);
sobel_plot(:, :, 1) = sobel_nrm;
sobel_plot(:, :, 2) = sobel_nrm;
sobel_plot(:, :, 3) = sobel_nrm;

depth_plot(:, :, 1) = data_prep.img_depth;
depth_plot(:, :, 2) = data_prep.img_depth;
depth_plot(:, :, 3) = data_prep.img_depth;

[n, ~] = size(plot_info.kp_tmp);
k(1:n, 1) = 2;

u = plot_info.kp_tmp(:, 1);
v = plot_info.kp_tmp(:, 2);
sobel_plot = insertShape(sobel_plot, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');
depth_plot = insertShape(depth_plot, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');

u = plot_info.kp_src(:, 1);
v = plot_info.kp_src(:, 2);
sobel_plot = insertShape(sobel_plot, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'red');
depth_plot = insertShape(depth_plot, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'red');

% figure
% imshow(sobel_plot)
% figure
% imshow(depth_plot)

% *************************************************** %
% ************ STATISTICS (EXPERIMENTS) ************* %
for i = 1:n
    x = plot_info.kp_tmp(i, 1) - plot_info.kp_src(i, 1);
    y = plot_info.kp_tmp(i, 2) - plot_info.kp_src(i, 2);
    [error, ~, ~] = cartesian2SphericalInDegrees(x, y, 0);
    results.error_signal = cat(1, results.error_signal, error);
    if error <= 3
        results.error_3 = results.error_3 + 1;
    elseif error <= 5
        results.error_5 = results.error_5 + 1;
    elseif error <= 8
        results.error_8 = results.error_8 + 1;
    elseif error <= 15
        results.error_15 = results.error_15 + 1;
    else
        results.error_x = results.error_x + 1;
    end
end

filename = strcat(experiments.fileout, num2str(experiments.id_dataset,'_%d'));
filename = strcat(filename, num2str(experiments.id_sample,'_%d.jpg'));
imwrite(sobel_plot, filename);
% *************************************************** %
% *************************************************** %
end