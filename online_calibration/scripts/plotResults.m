function plotResults(data, data_prep, matches, params, experiments)

[h, w, c] = size(data.image);
if c == 3
    sobel_plot(:, :, 1) = rgb2gray(data.image);
    sobel_plot(:, :, 2) = rgb2gray(data.image);
    sobel_plot(:, :, 3) = rgb2gray(data.image);
else
    sobel_plot(:, :, 1) = data.image;
    sobel_plot(:, :, 2) = data.image;
    sobel_plot(:, :, 3) = data.image;
end
sobel_nrm = data_prep.img_sobel / max(max(data_prep.img_sobel));
sobel_nrm = uint8(sobel_nrm * params.base);
sobel_plot2(:, :, 1) = sobel_nrm;
sobel_plot2(:, :, 2) = sobel_nrm;
sobel_plot2(:, :, 3) = sobel_nrm;

depth_plot = ind2rgb(uint8(data_prep.img_depth), jet(params.base+1));
depth_plot(:, :, 1) = depth_plot(:, :, 1) .* double(data_prep.img_depth > 0);
depth_plot(:, :, 2) = depth_plot(:, :, 2) .* double(data_prep.img_depth > 0);
depth_plot(:, :, 3) = depth_plot(:, :, 3) .* double(data_prep.img_depth > 0);
depth_plot = uint8(depth_plot * params.base);
depth_plot2 = depth_plot;
depth_plot2(:, :, :) = 0;
[v, u] = find(data_prep.img_discnt > params.threshold_dsc);
index = sub2ind([h w], v, u);
color2 = repmat(data_prep.img_discnt(index), 1, 3);
n = length(v);
kl(1:n, 1) = 0;
depth_plot2 = insertShape(depth_plot2, 'circle', [u, v, kl], 'LineWidth', 2, 'Color', color2);
color3 = color2;
color3(:, 1) = 0;
color3(:, 3) = 0;

final_plot = cat(1, sobel_plot, depth_plot);
final_plot2 = cat(1, sobel_plot2, depth_plot2);
final_plot3 = sobel_plot2;
final_plot3(:, :, :) = uint8(0);
% final_plot4 = sobel_plot;
% for i = 1:w
%     for j = 1:h
%         if (depth_plot(j, i, 1) > 1)
%             c = [depth_plot(j, i, 1) depth_plot(j, i, 2) depth_plot(j, i, 3)];
%             final_plot4 = insertShape(final_plot4, 'circle', [i, j, kl(1)], 'LineWidth', 2, 'Color', c);
%         end
%     end
% end

[m, ~] = size(matches.kp_tmp);
k(1:m*2, 1) = 3;

u_tmp = matches.kp_tmp(:, 1);
v_tmp = matches.kp_tmp(:, 2) + h;
u_tmp = cat(1, u_tmp, matches.pair_tmp(:, 1));
v_tmp = cat(1, v_tmp, matches.pair_tmp(:, 2) + h);
u_src = matches.kp_src(:, 1);
v_src = matches.kp_src(:, 2);
u_src = cat(1, u_src, matches.pair_src(:, 1));
v_src = cat(1, v_src, matches.pair_src(:, 2));

final_plot = insertShape(final_plot, 'line', [matches.kp_src(:, 1) matches.kp_src(:, 2) matches.pair_src(:, 1) matches.pair_src(:, 2)], 'LineWidth', 1, 'Color', 'blue');
final_plot = insertShape(final_plot, 'line', [u_tmp v_tmp u_src v_src], 'LineWidth', 1, 'Color', 'green');
final_plot = insertShape(final_plot, 'circle', [u_tmp, v_tmp, k], 'LineWidth', 3, 'Color', 'yellow');
final_plot = insertShape(final_plot, 'circle', [u_src, v_src, k], 'LineWidth', 3, 'Color', 'red');
final_plot2 = insertShape(final_plot2, 'line', [matches.kp_src(:, 1) matches.kp_src(:, 2) matches.pair_src(:, 1) matches.pair_src(:, 2)], 'LineWidth', 1, 'Color', 'blue');
final_plot2 = insertShape(final_plot2, 'line', [u_tmp v_tmp u_src v_src], 'LineWidth', 1, 'Color', 'green');
final_plot2 = insertShape(final_plot2, 'circle', [u_tmp, v_tmp, k], 'LineWidth', 3, 'Color', 'yellow');
final_plot2 = insertShape(final_plot2, 'circle', [u_src, v_src, k], 'LineWidth', 3, 'Color', 'red');
final_plot3 = insertShape(final_plot3, 'circle', [u, v, kl], 'LineWidth', 2, 'Color', color3);
final_plot3 = insertShape(final_plot3, 'line', [u_tmp v_tmp-h u_src v_src], 'LineWidth', 1, 'Color', 'yellow');
final_plot3 = insertShape(final_plot3, 'circle', [u_tmp, v_tmp-h, k], 'LineWidth', 3, 'Color', 'yellow');
final_plot3 = insertShape(final_plot3, 'circle', [u_src, v_src, k], 'LineWidth', 3, 'Color', 'red');

% % save image
% filename = strcat(experiments.fileout, num2str(experiments.id_dataset,'_%d'));
% filename = strcat(filename, num2str(experiments.id_sample,'_%d_1.jpg'));
% imwrite(final_plot, filename);
% filename = strcat(experiments.fileout, num2str(experiments.id_dataset,'_%d'));
% filename = strcat(filename, num2str(experiments.id_sample,'_%d_2.jpg'));
% imwrite(final_plot2, filename);
% filename = strcat(experiments.fileout, num2str(experiments.id_dataset,'_%d'));
% filename = strcat(filename, num2str(experiments.id_sample,'_%d_3.jpg'));
% imwrite(final_plot3, filename);

% plot image
close all
figure
imshow(final_plot)
figure
imshow(final_plot2)
figure
imshow(final_plot3)
% figure
% imshow(final_plot4)

% *************************************************** %
% ************ STATISTICS (EXPERIMENTS) ************* %
% for i = 1:n
%     x = matches.kp_tmp(i, 1) - matches.kp_src(i, 1);
%     y = matches.kp_tmp(i, 2) - matches.kp_src(i, 2);
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
%     x = matches.pair_tmp(i, 1) - matches.pair_src(i, 1);
%     y = matches.pair_tmp(i, 2) - matches.pair_src(i, 2);
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