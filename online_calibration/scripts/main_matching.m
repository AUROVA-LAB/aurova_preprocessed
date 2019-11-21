clear, clc, close all

%************* CONSTANTS ***************%
MIN_BLOB_AREA = 50;
INDEX_IMAGE = 280;
INDEX_BLOB = 3;
PLOT_RESULTS = 1;

%*********** read blobs images ************%
blob_filename = 'images/input/tr_02_dt_01/blob';
blob_filename = strcat(blob_filename, num2str(INDEX_IMAGE,'%d'));
gray_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_a.jpg'));
sobel_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_b.jpg'));
scans_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_c.jpg'));
intnst_filename = strcat(blob_filename, num2str(INDEX_BLOB,'_%d_d.jpg'));
gray = imread(gray_filename);
sobel = imread(sobel_filename);
scans = imread(scans_filename);
intnst = imread(intnst_filename);
[h, w, c] = size(intnst);

%************* edges detection *************%
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

% %***************** ICP ********************%
% THRESHOLD_TM = 10;
% THRESHOLD = 0;
% REDUCTION = 20;
% len = length(kp_x);
% kp_u = zeros(len, 1);
% kp_v = zeros(len, 1);
% for n = 1:len
%     template = discnt_image(kp_y(n):kp_y(n)+ROI_HEIGHT-1, kp_x(n):kp_x(n)+ROI_WIDTH-1);
%     [pt_y_tm, pt_x_tm] = find(template > THRESHOLD_TM); % cloud template
%     clear pt_z_tm;
%     pt_z_tm(1:length(pt_x_tm), 1) = double(0);
%     for i = 1:length(pt_x_tm)
%         pt_z_tm(i, 1) = (template(pt_y_tm(i, 1), pt_x_tm(i, 1)) - THRESHOLD_TM) / REDUCTION;
%     end
%     pt_x_tm = pt_x_tm + kp_x(n);
%     pt_y_tm = pt_y_tm + kp_y(n);
%     pt_xyz_tm = [pt_x_tm'; pt_y_tm'; pt_z_tm']';
%     pt_cloud_tm = pointCloud(pt_xyz_tm);
%     [pt_y, pt_x] = find(sobel_image(:, :, 1) > THRESHOLD); % cloud image
%     clear pt_z;
%     pt_z(1:length(pt_x), 1) = double(0);
%      for i = 1:length(pt_x)
%         pt_z(i, 1) = (sobel_image(pt_y(i, 1), pt_x(i, 1)) - THRESHOLD) / REDUCTION;
%     end
%     pt_xyz = [pt_x'; pt_y'; pt_z']';
%     pt_cloud = pointCloud(pt_xyz);
%     tform = pcregistericp(pt_cloud_tm, pt_cloud); % register ICP
%     trvec = tform2trvec(tform.T');
%     kp_u = kp_x(n) + trvec(1);
%     kp_v = kp_y(n) + trvec(2);
%     kp_u_plt = kp_u + ROI_WIDTH/2;
%     kp_v_plt = kp_v + ROI_HEIGHT/2;
%     kp_x_plt = kp_x(n) + ROI_WIDTH/2;
%     kp_y_plt = kp_y(n) + ROI_HEIGHT/2;
%     sobel_plot = insertShape(sobel_plot, 'line', [kp_u_plt kp_v_plt kp_x_plt kp_y_plt], 'LineWidth', 1, 'Color', 'yellow');
%     sobel_plot = insertShape(sobel_plot, 'circle', [kp_u_plt kp_v_plt RADIUS], 'LineWidth', 2, 'Color', 'red');
%     %match = sobel_image(kp_v:kp_v+ROI_HEIGHT-1, kp_u:kp_u+ROI_WIDTH-1);
% end

%*****************************************%
plot_gray_r = gray;
plot_gray_r(intnst > 20) = 255;
plot_gray_g = gray;
plot_gray_g(intnst > 20) = 0;
plot_gray_b = gray;
plot_gray_b(intnst > 20) = 0;
plot_gray_rgb = zeros(h, w, 3, 'uint8');
plot_gray_rgb(:, :, 1) = plot_gray_r;
plot_gray_rgb(:, :, 2) = plot_gray_g;
plot_gray_rgb(:, :, 3) = plot_gray_b;

plot_sobel_r = sobel;
plot_sobel_r(scans_edges > 20) = 255;
plot_sobel_g = sobel;
plot_sobel_g(scans_edges > 20) = 0;
plot_sobel_b = sobel;
plot_sobel_b(scans_edges > 20) = 0;
plot_sobel_rgb = zeros(h, w, 3, 'uint8');
plot_sobel_rgb(:, :, 1) = plot_sobel_r;
plot_sobel_rgb(:, :, 2) = plot_sobel_g;
plot_sobel_rgb(:, :, 3) = plot_sobel_b;

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