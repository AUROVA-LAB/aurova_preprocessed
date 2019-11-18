clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 64;
ROI_HEIGHT = 64;
STEPS = 5;
INI = 150;
NUM = INI;

%************** read images ***************%
image_filename_base = 'images/input/tr_01_dt_01/image';
scans_filename_base = 'images/input/tr_01_dt_01/scans';


image_filename_base_out = 'images/output/tr_01_dt_01/plot';
scans_filename_base_out = 'images/output/tr_01_dt_01/plot';

for index = INI:STEPS:NUM
image_filename = strcat(image_filename_base, num2str(index,'%d'));
scans_filename = strcat(scans_filename_base, num2str(index,'%d'));
image_filename = strcat(image_filename, '.jpg');
scans_filename = strcat(scans_filename, '.jpg');
image = imread(image_filename);
scans = imread(scans_filename);
[h, w, c] = size(image);

scans_plot = scans;
image_plot = image;

%********* preprocess image ***************%
scans_z_plot = scans(:, :, 2);
scans_i_plot = scans(:, :, 1);

%*********** representation ***************%
close all
movegui(figure,'southeast');
imshow(scans_z_plot);
movegui(figure,'southwest');
imshow(scans_i_plot);
movegui(figure,'northwest');
imshow(scans_plot);
movegui(figure,'northeast');
imshow(image_plot);

%************* save results ****************%
% image_filename = strcat(image_filename_base_out, num2str(index,'%d'));
% image_filename = strcat(image_filename, '_a.jpg');
% imwrite(image_plot, image_filename);
% scans_filename = strcat(scans_filename_base_out, num2str(index,'%d'));
% scans_filename = strcat(scans_filename, '_d.jpg');
% imwrite(scans_plot, scans_filename);
% scans_filename = strcat(scans_filename_base_out, num2str(index,'%d'));
% scans_filename = strcat(scans_filename, '_e.jpg');
% imwrite(entropy_map_plot_g, scans_filename);
% scans_filename = strcat(scans_filename_base_out, num2str(index,'%d'));
% scans_filename = strcat(scans_filename, '_m.jpg');
% imwrite(entropy_color_map, scans_filename);

end