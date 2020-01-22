clear, clc, close all

%************* CONSTANTS ***************%
ROI_WIDTH = 64;
ROI_HEIGHT = 64;
MIN_BLOB_AREA = 2000;
STEPS = 5;
INI = 280;
NUM = INI; % 480
SAVE_RESULTS = 0;
SAVE_BLOBS = 1;
PLOT_RESULTS = 1;

%*************** STRUCTS ****************%

st_blob = struct('image_blob',	[], ...
                          'label',            0, ...
                          'centroid',       [], ...
                          'area',             0, ...
                          'bbox',            [], ...
                          'cluster_id',     0);

%************** read images ***************%
image_filename_base = 'images/input/tr_01_dt_01/image';
scans_filename_base = 'images/input/tr_01_dt_01/scans';
intensity_filename_base = 'images/input/tr_01_dt_01/intensity';
filename_base_out = 'images/output/tr_01_dt_01/result';
filename_blobs_out = 'images/input/tr_02_dt_01/blob';

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
image_gray_g = imgaussfilt(image_gray, 2.0);
[sobel_mag, solbel_dir] = imgradient(image_gray_g, 'sobel');

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
            blob.centroid = centroid(m, :);
            blob.area = area(m);
            blob.bbox = bbox(m, :);
            blob.cluster_id = n;
            blobs_per_seg = [blobs_per_seg; blob];
        end
    end
end

%********** segmentation image ***********%
d = 10;
offbox = 40;
plot = [];
plot_sobel = [];
plot_scans = [];
plot_intensity = [];
for n = 1:length(blobs_per_seg)
    
    mask = blobs_per_seg(n).image_blob;
    for m = 1:d
        mask = imageDilate(mask, 3, 2);
    end
    kn = blobs_per_seg(n).cluster_id;
    
%     mask(1:h, 1:w) = 0;
%     x1 = blobs_per_seg(n).bbox(1) - offbox;
%     x2 = blobs_per_seg(n).bbox(1) + blobs_per_seg(n).bbox(3) + offbox - 1;
%     y1 = blobs_per_seg(n).bbox(2) - offbox;
%     y2 = blobs_per_seg(n).bbox(2) + blobs_per_seg(n).bbox(4) + offbox - 1;
%     
%     if x1 < 1
%         x1 = 1;
%     end
%     if x2 > w
%         x2 = w;
%     end
%     if y1 < 1
%         y1 = 1;
%     end
%     if y2 > h
%         y2 = h;
%     end
%     
%     mask(y1:y2, x1:x2) = 1;
%     mask = activecontour(image, mask);
    
    %******** generate images for plot **********%
    plot = cat(3, plot, image_gray .* uint8(mask));
    plot_sobel = cat(3, plot_sobel, uint8(sobel_mag) .* uint8(mask));
    plot_scans = cat(3, plot_scans, uint8(blobs_per_seg(n).image_blob) .* uint8(scans_seg_slices(:, :, kn)) * 255);
    plot_intensity = cat(3, plot_intensity, intensity(:, :, 1) .* uint8(blobs_per_seg(n).image_blob) .* uint8(scans_seg_slices(:, :, kn)));
end

 
%************* plot images *****************%
if PLOT_RESULTS
    close all
    movegui(figure,'southwest');
    montage(plot);
    curr_montage_a = getframe(gca);
    movegui(figure,'southeast');
    montage(plot_sobel);
    curr_montage_b = getframe(gca);
    movegui(figure,'northwest');
    montage(plot_scans);
    curr_montage_c = getframe(gca);
    movegui(figure,'northeast');
    montage(plot_intensity);
    curr_montage_d = getframe(gca);
end

%************* save results ****************%
if SAVE_RESULTS
    filename = strcat(filename_base_out, num2str(index,'%d'));
    filename = strcat(filename, '_a.jpg');
    imwrite(curr_montage_a.cdata, filename);
    filename = strcat(filename_base_out, num2str(index,'%d'));
    filename = strcat(filename, '_b.jpg');
    imwrite(curr_montage_b.cdata, filename);
    filename = strcat(filename_base_out, num2str(index,'%d'));
    filename = strcat(filename, '_c.jpg');
    imwrite(curr_montage_c.cdata, filename);
    filename = strcat(filename_base_out, num2str(index,'%d'));
    filename = strcat(filename, '_d.jpg');
    imwrite(curr_montage_d.cdata, filename);
end

if SAVE_BLOBS
    [h, w, c] = size(plot);
    for n = 1:c
        filename = strcat(filename_blobs_out, num2str(index,'%d'));
        filename = strcat(filename,  num2str(n,'_%d_a.jpg'));
        imwrite(plot(:, :, n), filename);
        
        filename = strcat(filename_blobs_out, num2str(index,'%d'));
        filename = strcat(filename,  num2str(n,'_%d_b.jpg'));
        imwrite(plot_sobel(:, :, n), filename);
        
        filename = strcat(filename_blobs_out, num2str(index,'%d'));
        filename = strcat(filename,  num2str(n,'_%d_c.jpg'));
        imwrite(plot_scans(:, :, n), filename);
        
        filename = strcat(filename_blobs_out, num2str(index,'%d'));
        filename = strcat(filename,  num2str(n,'_%d_d.jpg'));
        imwrite(plot_intensity(:, :, n), filename);
    end
end

end