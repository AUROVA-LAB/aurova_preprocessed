clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 128;
ROI_HEIGHT = 128;
STEP_IMAGES = 5;
INI_IMAGES = 150;
NUM_IMAGES = INI_IMAGES; %495;
W_SOBEL = 1.5;
RADIUS = 2;

%************** read images ***************%
sobel_filename_base = 'images/input/tr_01_dt_01/sobel';
discnt_filename_base = 'images/input/tr_01_dt_01/discnt';
sobel_filename_base_out = 'images/output/tr_01_dt_01/plot';
discnt_filename_base_out = 'images/output/tr_01_dt_01/plot';

for index = INI_IMAGES:STEP_IMAGES:NUM_IMAGES
sobel_filename = strcat(sobel_filename_base, num2str(index,'%d'));
discnt_filename = strcat(discnt_filename_base, num2str(index,'%d'));
sobel_filename = strcat(sobel_filename, '.jpg');
discnt_filename = strcat(discnt_filename, '.jpg');
sobel_image = imread(sobel_filename);
discnt_image = imread(discnt_filename);
[h, w, c] = size(sobel_image);

%********* preprocess image ***************%
sobel_image = sobel_image .* W_SOBEL;
% sobel_image_g = imgaussfilt(sobel_image, 5.0);
discnt_image_g = imgaussfilt(discnt_image, 5.0);
% sobel_image = sobel_image* 0.5 + sobel_image_g;
discnt_image_g = discnt_image * 0.5 + discnt_image_g;

%******** image entropy calculation ********%
[entropy_map, entropy_map_mask] = imageEntropy(discnt_image_g, ROI_WIDTH, ROI_HEIGHT);
entropy_map = entropy_map(1:h-ROI_HEIGHT-1, 1:w-ROI_WIDTH-1);
entropy_map_mask = entropy_map_mask(1:h-ROI_HEIGHT-1, 1:w-ROI_WIDTH-1);
entropy_map_plot = entropy_map - min(min(entropy_map));
entropy_map_plot = entropy_map_plot/max(max(entropy_map_plot));
entropy_color_map = ind2rgb(uint8(entropy_map_plot*256), jet(256));

%******* local maximun calculation ********%
entropy_map_plot_g = imgaussfilt(entropy_map_plot, 6.0) .* entropy_map_mask;
maximums = imregionalmax(entropy_map_plot_g);
[kp_y, kp_x] = find(maximums > 0); % keypoints
% maximums = max(max(entropy_map_plot_g));
% [kp_y, kp_x] = find(entropy_map_plot_g == maximums); % keypoints
clear k;
k(1:length(kp_x), 1) = RADIUS;
kp_x_plt = kp_x + ROI_WIDTH/2;
kp_y_plt = kp_y + ROI_HEIGHT/2;
entropy_map_plot_g = insertShape(entropy_map_plot_g, 'circle', [kp_x, kp_y, k], 'LineWidth', 2, 'Color', 'green');
discnt_plot = insertShape(discnt_image, 'circle', [kp_x_plt, kp_y_plt, k], 'LineWidth', 2, 'Color', 'green');
sobel_plot = insertShape(sobel_image, 'circle', [kp_x_plt, kp_y_plt, k], 'LineWidth', 2, 'Color', 'green');

%*****************************************%
THRESHOLD_TM = 10;
THRESHOLD = 0;
REDUCTION = 20;
len = length(kp_x);
kp_u = zeros(len, 1);
kp_v = zeros(len, 1);
for n = 1:len
    template = discnt_image(kp_y(n):kp_y(n)+ROI_HEIGHT-1, kp_x(n):kp_x(n)+ROI_WIDTH-1);
    [pt_y_tm, pt_x_tm] = find(template > THRESHOLD_TM); % cloud template
    clear pt_z_tm;
    pt_z_tm(1:length(pt_x_tm), 1) = double(0);
    for i = 1:length(pt_x_tm)
        pt_z_tm(i, 1) = (template(pt_y_tm(i, 1), pt_x_tm(i, 1)) - THRESHOLD_TM) / REDUCTION;
    end
    pt_x_tm = pt_x_tm + kp_x(n);
    pt_y_tm = pt_y_tm + kp_y(n);
    pt_xyz_tm = [pt_x_tm'; pt_y_tm'; pt_z_tm']';
    pt_cloud_tm = pointCloud(pt_xyz_tm);
    [pt_y, pt_x] = find(sobel_image(:, :, 1) > THRESHOLD); % cloud image
    clear pt_z;
    pt_z(1:length(pt_x), 1) = double(0);
     for i = 1:length(pt_x)
        pt_z(i, 1) = (sobel_image(pt_y(i, 1), pt_x(i, 1)) - THRESHOLD) / REDUCTION;
    end
    pt_xyz = [pt_x'; pt_y'; pt_z']';
    pt_cloud = pointCloud(pt_xyz);
    tform = pcregistericp(pt_cloud_tm, pt_cloud); % register ICP
    trvec = tform2trvec(tform.T');
    kp_u = kp_x(n) + trvec(1);
    kp_v = kp_y(n) + trvec(2);
    kp_u_plt = kp_u + ROI_WIDTH/2;
    kp_v_plt = kp_v + ROI_HEIGHT/2;
    kp_x_plt = kp_x(n) + ROI_WIDTH/2;
    kp_y_plt = kp_y(n) + ROI_HEIGHT/2;
    sobel_plot = insertShape(sobel_plot, 'line', [kp_u_plt kp_v_plt kp_x_plt kp_y_plt], 'LineWidth', 1, 'Color', 'yellow');
    sobel_plot = insertShape(sobel_plot, 'circle', [kp_u_plt kp_v_plt RADIUS], 'LineWidth', 2, 'Color', 'red');
    %match = sobel_image(kp_v:kp_v+ROI_HEIGHT-1, kp_u:kp_u+ROI_WIDTH-1);
end

%************ KL divergence **************%
% len = length(kp_x);
% kp_u = zeros(len, 1);
% kp_v = zeros(len, 1);
% for n = 1:len
%     template = discnt_image(kp_y(n):kp_y(n)+ROI_HEIGHT-1, kp_x(n):kp_x(n)+ROI_WIDTH-1);
%     kl_map = imageKLDivergence(sobel_image, template, kp_x(n), kp_y(n), ROI_WIDTH/2, ROI_HEIGHT/2);
%     min_val = min(kl_map(kl_map > 0));
%     [v, u] = find(kl_map == min_val); % keypoints
%     kp_u(n) = u;
%     kp_v(n) = v;
%     sobel_plot = insertShape(sobel_plot, 'line', [u+ROI_WIDTH/2 v+ROI_HEIGHT/2 kp_x(n)+ROI_WIDTH/2 kp_y(n)+ROI_HEIGHT/2], 'LineWidth', 1, 'Color', 'yellow');
%     sobel_plot = insertShape(sobel_plot, 'circle', [u+ROI_WIDTH/2 v+ROI_HEIGHT/2 RADIUS], 'LineWidth', 2, 'Color', 'red');
%     match = sobel_image(v:v+ROI_HEIGHT-1, u:u+ROI_WIDTH-1);
% end

%*********** representation ***************%
close all
movegui(figure,'southeast');
imshow(entropy_map_plot_g);
movegui(figure,'southwest');
imshow(entropy_color_map);
movegui(figure,'northwest');
imshow(discnt_plot);
movegui(figure,'northeast');
imshow(sobel_plot);

%************* save results ****************%
% sobel_filename = strcat(sobel_filename_base_out, num2str(index,'%d'));
% sobel_filename = strcat(sobel_filename, '_a.jpg');
% imwrite(sobel_plot, sobel_filename);
% discnt_filename = strcat(discnt_filename_base_out, num2str(index,'%d'));
% discnt_filename = strcat(discnt_filename, '_d.jpg');
% imwrite(discnt_plot, discnt_filename);
% discnt_filename = strcat(discnt_filename_base_out, num2str(index,'%d'));
% discnt_filename = strcat(discnt_filename, '_e.jpg');
% imwrite(entropy_map_plot_g, discnt_filename);
% discnt_filename = strcat(discnt_filename_base_out, num2str(index,'%d'));
% discnt_filename = strcat(discnt_filename, '_m.jpg');
% imwrite(entropy_color_map, discnt_filename);

end