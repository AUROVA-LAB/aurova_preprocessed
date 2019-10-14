clear, clc, close all
constants

%************** read images ***************%
filename_sobel_base = 'images/tr_01/sobel';
filename_edges_base = 'images/tr_01/edges';

for index = INI_IMAGES:STEP_IMAGES:NUM_IMAGES
filename_sobel = strcat(filename_sobel_base, num2str(index,'%d'));
filename_edges = strcat(filename_edges_base, num2str(index,'%d'));
filename_sobel = strcat(filename_sobel, '.jpg');
filename_edges = strcat(filename_edges, '.jpg');
image_sobel = imread(filename_sobel);
image_edges = imread(filename_edges);
image_sobel = image_sobel .* W_SOBEL;

%******** image entropy calculation ********%
[image_entropy, image_entropy_max] = imageEntropy(image_edges, MASK_WIDTH, MASK_HEIGHT);
max_val = max(max(image_entropy));
[y, x] = find(image_entropy == max_val);
plot_edges = insertShape(image_edges, 'rectangle', [x(1) y(1) MASK_WIDTH MASK_HEIGHT], 'LineWidth', 2, 'Color', 'green');
plot_sobel = insertShape(image_sobel, 'rectangle', [x(1) y(1) MASK_WIDTH MASK_HEIGHT], 'LineWidth', 2, 'Color', 'green');

%********* template matching **************%
template = image_edges(y(1):y(1)+MASK_HEIGHT, x(1):x(1)+MASK_WIDTH);
plot_match = tmc(template, image_sobel);
[y, x] = location(image_sobel, template, plot_match);
plot_sobel = insertShape(plot_sobel, 'rectangle', [x(1) y(1) MASK_WIDTH MASK_HEIGHT], 'LineWidth', 2, 'Color', 'red');

%*********** representation ***************%
close all
movegui(figure,'southeast');
imshow(image_entropy);
movegui(figure,'southwest');
imshow(image_entropy_max);
movegui(figure,'northeast');
imshow(plot_sobel);
movegui(figure,'northwest');
imshow(plot_edges);
end