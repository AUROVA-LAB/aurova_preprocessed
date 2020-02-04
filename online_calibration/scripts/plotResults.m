function results = plotResults(data_prep, plot_info, params, experiments, results)

sobel_nrm = data_prep.img_sobel / max(max(data_prep.img_sobel));
sobel_nrm = uint8(sobel_nrm * params.base);
sobel_plot(:, :, 1) = sobel_nrm;
sobel_plot(:, :, 2) = sobel_nrm;
sobel_plot(:, :, 3) = sobel_nrm;

depth_plot(:, :, 1) = 0;
depth_plot(:, :, 2) = 0;
depth_plot(:, :, 3) = data_prep.img_depth;
depth_plot(:, :, 2) = double(data_prep.img_discnt > 20) * params.base;
depth_plot(:, :, 3) = depth_plot(:, :, 3) - (uint8(data_prep.img_discnt > 0) * params.base);
% depth_plot = ind2rgb(uint8(data_prep.img_depth), jet(params.base+1));
% depth_plot(:, :, 1) = depth_plot(:, :, 1) .* double(data_prep.img_depth > 0);
% depth_plot(:, :, 2) = depth_plot(:, :, 2) .* double(data_prep.img_depth > 0);
% depth_plot(:, :, 3) = depth_plot(:, :, 3) .* double(data_prep.img_depth > 0);
% depth_plot = uint8(depth_plot * params.base);

final_plot = cat(1, sobel_plot, depth_plot);

[n, ~] = size(plot_info.kp_tmp);
[mm, ~] = size(sobel_plot);
k(1:n*2, 1) = 2;

u_tmp = plot_info.kp_tmp(:, 1);
v_tmp = plot_info.kp_tmp(:, 2) + mm;
u_tmp = cat(1, u_tmp, plot_info.pair_tmp(:, 1));
v_tmp = cat(1, v_tmp, plot_info.pair_tmp(:, 2) + mm);
u_src = plot_info.kp_src(:, 1);
v_src = plot_info.kp_src(:, 2);
u_src = cat(1, u_src, plot_info.pair_src(:, 1));
v_src = cat(1, v_src, plot_info.pair_src(:, 2));

final_plot = insertShape(final_plot, 'line', [u_tmp v_tmp u_src v_src], 'LineWidth', 1, 'Color', 'green');
final_plot = insertShape(final_plot, 'circle', [u_tmp, v_tmp, k], 'LineWidth', 2, 'Color', 'yellow');
final_plot = insertShape(final_plot, 'circle', [u_src, v_src, k], 'LineWidth', 2, 'Color', 'red');

% save image
filename = strcat(experiments.fileout, num2str(experiments.id_dataset,'_%d'));
filename = strcat(filename, num2str(experiments.id_sample,'_%d.jpg'));
imwrite(final_plot, filename);

% *************************************************** %
% ************ STATISTICS (EXPERIMENTS) ************* %
% for i = 1:n
%     x = plot_info.kp_tmp(i, 1) - plot_info.kp_src(i, 1);
%     y = plot_info.kp_tmp(i, 2) - plot_info.kp_src(i, 2);
%     [error, ~, ~] = cartesian2SphericalInDegrees(x, y, 0);
%     results.error_signal = cat(1, results.error_signal, error);
%     if error <= 3
%         results.error_3 = results.error_3 + 1;
%     elseif error <= 5
%         results.error_5 = results.error_5 + 1;
%     elseif error <= 8
%         results.error_8 = results.error_8 + 1;
%     elseif error <= 15
%         results.error_15 = results.error_15 + 1;
%     else
%         results.error_x = results.error_x + 1;
%     end
%     
%     x = plot_info.pair_tmp(i, 1) - plot_info.pair_src(i, 1);
%     y = plot_info.pair_tmp(i, 2) - plot_info.pair_src(i, 2);
%     [error, ~, ~] = cartesian2SphericalInDegrees(x, y, 0);
%     results.error_signal = cat(1, results.error_signal, error);
%     if error <= 3
%         results.error_3 = results.error_3 + 1;
%     elseif error <= 5
%         results.error_5 = results.error_5 + 1;
%     elseif error <= 8
%         results.error_8 = results.error_8 + 1;
%     elseif error <= 15
%         results.error_15 = results.error_15 + 1;
%     else
%         results.error_x = results.error_x + 1;
%     end
% end
% *************************************************** %
% *************************************************** %
end