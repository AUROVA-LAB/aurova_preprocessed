clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 64;
ROI_HEIGHT = 64;
MIN_BLOB_AREA = 2000;
STEPS = 5;
INI = 150;
NUM = 600; % 480

%*************** STRUCTS ****************%

st_blob = struct('image_blob',	[], ...
                          'label',            0, ...
                          'centroid',       [], ...
                          'area',             0, ...
                          'bbox',            []);

%************** read images ***************%
image_filename_base = 'images/input/tr_01_dt_07/image';
scans_filename_base = 'images/input/tr_01_dt_07/scans';
intensity_filename_base = 'images/input/tr_01_dt_07/intensity';
filename_base_out = 'images/output/tr_01_dt_07/result';

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
image_gray = rgb2gray(image);
% [sobel_mag, solbel_dir] = imgradient(image_gray, 'sobel');

%********* segmentation of scan im ********%
k = 6;
scans_seg = imageKmeansSeg(scans, k);
scans_mix_seg = imageKmeansSeg(scans_mix, k);
image_seg = imageKmeansSeg(image, k);
scans_seg_slices = zeros(h, w, k);
intnst_seg_slices = zeros(h, w, k);
for n = 1:k
    scans_seg_slices(:, :, n) = scans_mix_seg == n;
end
d = 4;
scans_seg_dilated = scans_seg_slices;
for n = 1:d
    scans_seg_dilated = imageDilate(scans_seg_dilated, 3, 2);
end
e = 4;
for n = 1:e
    scans_seg_dilated = imageErode(scans_seg_dilated, 3, 2);
end

%************ blobs extraction *************%
hblob = vision.BlobAnalysis;
hblob.release();
hblob.LabelMatrixOutputPort  = true;
blob = st_blob; 
blobs_per_seg = []; 
for n = 1:k
    [area, centroid, bbox, labels] = step(hblob, logical(scans_seg_dilated(:, :, n)));
    for m = 1:length(area)
        if area(m) > MIN_BLOB_AREA
            blob.image_blob = labels == m;
            blob.label = m;
            blob.centroid = centroid(m);
            blob.area = area(m);
            blob.bbox = bbox(m);
            blobs_per_seg = [blobs_per_seg; blob];
        end
    end
end

%********** segmentation image ***********%
d = 10;
plot = [];
plot_scans = [];
plot_intensity = [];
for n = 1:length(blobs_per_seg)
    mask = blobs_per_seg(n).image_blob;
    for m = 1:d
        mask = imageDilate(mask, 3, 2);
    end
    segment = activecontour(image, mask);
    plot = cat(3, plot, image_gray .* uint8(segment));
    plot_scans = cat(3, plot_scans, scans(:, :, 1) .* uint8(blobs_per_seg(n).image_blob));
    plot_intensity = cat(3, plot_intensity, intensity(:, :, 1) .* uint8(blobs_per_seg(n).image_blob));
end

%******** generate images for plot **********%
% plot_blobs = [];
% for n = 1:length(blobs_per_seg)
%     plot_blobs = cat(3, plot_blobs, blobs_per_seg(n).image_blob*255);
% end
% plot_blobs = cat(3, plot_blobs, image_gray);
 
%************* plot images *****************%
close all
movegui(figure,'southwest');
montage(plot);
curr_montage_a = getframe(gca);
movegui(figure,'northwest');
montage(plot_scans);
curr_montage_b = getframe(gca);
movegui(figure,'northeast');
montage(plot_intensity);
curr_montage_c = getframe(gca);

%************* save results ****************%
filename = strcat(filename_base_out, num2str(index,'%d'));
filename = strcat(filename, '_a.jpg');
imwrite(curr_montage_a.cdata, filename);
filename = strcat(filename_base_out, num2str(index,'%d'));
filename = strcat(filename, '_b.jpg');
imwrite(curr_montage_b.cdata, filename);
filename = strcat(filename_base_out, num2str(index,'%d'));
filename = strcat(filename, '_c.jpg');
imwrite(curr_montage_c.cdata, filename);


end