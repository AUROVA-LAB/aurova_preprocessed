clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 64;
ROI_HEIGHT = 64;
STEP_IMAGES = 10;
INI_IMAGES = 250;
NUM_IMAGES = INI_IMAGES; %987;
W_SOBEL = 1.0;

%************** read images ***************%
sobel_filename_base = 'images/tr_01/sobel';
discnt_filename_base = 'images/tr_01/edges';

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
sobel_image_g = imgaussfilt(sobel_image, 5.0);
discnt_image_g = imgaussfilt(discnt_image, 5.0);
sobel_image = sobel_image* 0.5 + sobel_image_g;
discnt_image = discnt_image * 0.5 + discnt_image_g;

%******** image entropy calculation ********%
[d_entropy_map, d_entropy_map_mask] = imageEntropy(discnt_image, ROI_WIDTH, ROI_HEIGHT);
max_val = max(max(d_entropy_map));
[y, x] = find(d_entropy_map == max_val);
discnt_plot = insertShape(discnt_image, 'rectangle', [x(1) y(1) ROI_WIDTH ROI_HEIGHT], 'LineWidth', 2, 'Color', 'green');
sobel_plot = insertShape(sobel_image, 'rectangle', [x(1) y(1) ROI_WIDTH ROI_HEIGHT], 'LineWidth', 2, 'Color', 'green');
template = discnt_image(y(1):y(1)+ROI_HEIGHT-1, x(1):x(1)+ROI_WIDTH-1);

%********** cross correlation **************%
% corr_map = imageCrossCorrelation(sobel_image, template);
% max_val = max(max(corr_map));
% [y, x] = find(corr_map == max_val);
% sobel_plot = insertShape(sobel_plot, 'rectangle', [x(1) y(1) ROI_WIDTH ROI_HEIGHT], 'LineWidth', 2, 'Color', 'red');

%************ KL divergence **************%
kl_map = imageKLDivergence(sobel_image, template);
kl_map = kl_map(1:h-ROI_HEIGHT-1, 1:w-ROI_WIDTH-1);
min_val = min(min(kl_map));
[y, x] = find(kl_map == min_val);
sobel_plot = insertShape(sobel_plot, 'rectangle', [x(1) y(1) ROI_WIDTH ROI_HEIGHT], 'LineWidth', 2, 'Color', 'red');
match = sobel_image(y(1):y(1)+ROI_HEIGHT-1, x(1):x(1)+ROI_WIDTH-1);
kl_map_plot = kl_map/max(max(kl_map));
kl_map_plot = kl_map_plot - min(min(kl_map_plot));
kl_map_plot = kl_map_plot/max(max(kl_map_plot));

%********** mutual information ************%
% mi_map = imageMutualInformation(sobel_image, template);
% mi_map = mi_map(1:h-ROI_HEIGHT-1, 1:w-ROI_WIDTH-1);
% max_val = max(max(mi_map));
% [y, x] = find(mi_map == max_val);
% sobel_plot = insertShape(sobel_plot, 'rectangle', [x(1) y(1) ROI_WIDTH ROI_HEIGHT], 'LineWidth', 2, 'Color', 'red');
% match = sobel_image(y(1):y(1)+ROI_HEIGHT-1, x(1):x(1)+ROI_WIDTH-1);
% mi_map_plot = mi_map/max(max(mi_map));
% mi_map_plot = mi_map_plot - min(min(mi_map_plot));
% mi_map_plot = mi_map_plot/max(max(mi_map_plot));

%*********** representation ***************%
close all
movegui(figure,'southeast');
imshow(kl_map_plot);
movegui(figure,'southwest');
imshow(d_entropy_map_mask);
movegui(figure,'northwest');
imshow(discnt_plot);
movegui(figure,'northeast');
imshow(sobel_plot);
end