clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 64;
ROI_HEIGHT = 64;
STEPS = 5;
INI = 280;
NUM = INI; % 480

%************** read images ***************%
image_filename_base = 'images/input/tr_01_dt_01/image';
scans_filename_base = 'images/input/tr_01_dt_01/scans';
intensity_filename_base = 'images/input/tr_01_dt_01/intensity';
image_filename_base_out = 'images/output/tr_01_dt_01/plot';
scans_filename_base_out = 'images/output/tr_01_dt_01/plot';

for index = INI:STEPS:NUM
image_filename = strcat(image_filename_base, num2str(index,'%d'));
scans_filename = strcat(scans_filename_base, num2str(index,'%d'));
intensity_filename = strcat(intensity_filename_base, num2str(index,'%d'));
image_filename = strcat(image_filename, '.jpg');
scans_filename = strcat(scans_filename, '.jpg');
intensity_filename = strcat(intensity_filename, '.jpg');
image = imread(image_filename);
scans = imread(scans_filename);
intensity = imread(intensity_filename);
scans_mix = scans;
scans_mix(:, :, 1) = intensity(:, :, 1);
scans_mix(:, :, 3) = intensity(:, :, 1);
[h, w, c] = size(image);

%*********** preprocess images ************%
% image_gray = rgb2gray(image);
% [sobel_mag, solbel_dir] = imgradient(image_gray, 'sobel');

%********* segmentation of images *********%
k = 6;
image_seg = imageKmeansSeg(image, k);
scans_seg = imageKmeansSeg(scans, k);
scans_mix_seg = imageKmeansSeg(scans_mix, k);
intensity_seg = imageKmeansSeg(intensity, k);
scans_seg_k = zeros(h, w, k);
for n = 1:k
    scans_seg_k(:, :, n) = scans_seg == n;
end

%******** generate images for plot **********%

%************* plot images *****************%
close all
montage(scans_seg_k);
movegui(figure,'southeast');
imshow(image);
% movegui(figure,'southeast');
% imshow(scans_mix_seg_plot);
% movegui(figure,'southwest');
% imshow(scans_mix_plot);
% movegui(figure,'northwest');
% imshow(scans_seg_plot);
% movegui(figure,'northeast');
% imshow(image_seg_plot);

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