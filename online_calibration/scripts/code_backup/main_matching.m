clear, clc, close all

%************* CONSTANTS ***************%
MIN_BLOB_AREA = 50;
INDEX_IMAGE = 280;
INDEX_BLOB = 1;
PLOT_RESULTS = 1;

%*********** read blobs images ************%
blob_filename = 'images/input/tr_02_dt_01/blob';
image_filename = 'images/input/tr_01_dt_01/image';
blob_filename = strcat(blob_filename, num2str(INDEX_IMAGE,'%d'));
image_filename = strcat(image_filename, num2str(INDEX_IMAGE,'%d.jpg'));
gray_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_a.jpg'));
sobel_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_b.jpg'));
scans_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_c.jpg'));
intnst_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_d.jpg'));
gray = imread(gray_filename);
sobel = imread(sobel_filename);
scans = imread(scans_filename);
intnst = imread(intnst_filename);
image = imread(image_filename);
[h, w, c] = size(intnst);

%************* edges detection *************%
image_gray = rgb2gray(image);
canny = edge(gray, 'canny', 0.2);
canny = uint8(canny) * 255;
scans_dilated = imageDilate(scans > 20, 3, 3);
scans_dilated = imageDilate(scans_dilated, 3, 3);
scans_dilated = imageDilate(scans_dilated, 3, 3);
hblob = vision.BlobAnalysis;
hblob.release();
hblob.LabelMatrixOutputPort  = true;
[area, centroid, bbox, labels] = step(hblob, scans_dilated);
scans_filtered = zeros(h, w, 1, 'logical');
for m = 1:length(area)
    if area(m) > MIN_BLOB_AREA
        scans_filtered = scans_filtered + (labels == m);
    end
end
scans_edges = imgradient(double(scans_filtered)*255, 'sobel');
% scans_edges = edge(double(scans_filtered)*255, 'canny', 0.1);
% scans_edges = double(scans_edges) * 255;

%***************** ICP ********************%
THRESHOLD_TM = 20;
THRESHOLD = 20;
REDUCTION = 20;
[pt_y_tm, pt_x_tm] = find(scans_edges > THRESHOLD_TM); % cloud template
clear pt_z_tm;
pt_z_tm(1:length(pt_x_tm), 1) = double(0);
% for i = 1:length(pt_x_tm)
%     pt_z_tm(i, 1) = (scans_edges(pt_y_tm(i, 1), pt_x_tm(i, 1)) - THRESHOLD_TM) / REDUCTION;
% end
pt_xyz_tm = [pt_x_tm'; pt_y_tm'; pt_z_tm']';
pt_cloud_tm = pointCloud(pt_xyz_tm);
[pt_y, pt_x] = find(canny > THRESHOLD); % cloud image
clear pt_z;
pt_z(1:length(pt_x), 1) = double(0);
%  for i = 1:length(pt_x)
%     pt_z(i, 1) = (canny(pt_y(i, 1), pt_x(i, 1)) - THRESHOLD) / REDUCTION;
% end
pt_xyz = [pt_x'; pt_y'; pt_z']';
pt_cloud = pointCloud(pt_xyz);
t_form = pcregistericp(pt_cloud_tm, pt_cloud); % register ICP
tr_vec = tform2trvec(t_form.T');
pt_cloud_match = pctransform(pt_cloud_tm, t_form);


%*****************************************%
plot_gray_r = image_gray;
plot_gray_r(scans_edges > 20) = 255;
plot_gray_g = image_gray;
plot_gray_g(scans_edges > 20) = 0;
plot_gray_b = image_gray;
plot_gray_b(scans_edges > 20) = 0;
plot_gray_rgb = zeros(h, w, 3, 'uint8');
plot_gray_rgb(:, :, 1) = plot_gray_r;
plot_gray_rgb(:, :, 2) = plot_gray_g;
plot_gray_rgb(:, :, 3) = plot_gray_b;

plot_sobel_r = canny;
plot_sobel_r(scans_edges > 20) = 255;
plot_sobel_g = canny;
plot_sobel_g(scans_edges > 20) = 0;
plot_sobel_b = canny;
plot_sobel_b(scans_edges > 20) = 0;
plot_sobel_rgb = zeros(h, w, 3, 'uint8');
plot_sobel_rgb(:, :, 1) = plot_sobel_r;
plot_sobel_rgb(:, :, 2) = plot_sobel_g;
plot_sobel_rgb(:, :, 3) = plot_sobel_b;

for n = 1:pt_cloud_match.Count
    u = round(pt_cloud_match.Location(n, 1));
    v = round(pt_cloud_match.Location(n, 2));
    if u > 0 && v > 0
        plot_sobel_rgb(v, u, 1) = 0;
        plot_sobel_rgb(v, u, 2) = 255;
        plot_sobel_rgb(v, u, 3) = 0;
        plot_gray_rgb(v, u, 1) = 0;
        plot_gray_rgb(v, u, 2) = 255;
        plot_gray_rgb(v, u, 3) = 0;
    end
end

plot_originals = cat(3, gray, sobel, intnst, scans);

%************** plot results ****************%
if PLOT_RESULTS
    close all
    movegui(figure,'southeast');
    montage(plot_originals);
    movegui(figure,'northwest');
    imshow(plot_gray_rgb);
    movegui(figure,'northeast');
    imshow(plot_sobel_rgb);
end