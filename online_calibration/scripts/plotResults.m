function plotResults(data_prep, plot_info, params)

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

figure
imshow(sobel_plot)
figure
imshow(depth_plot)
end