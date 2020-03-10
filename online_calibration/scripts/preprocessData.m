function data_prep = preprocessData(data, params)

data_prep = [];

% preprocess camera data image
[~, ~, c] = size(data.image);
if c == 3
    image_gray = rgb2gray(data.image);
else
    image_gray = data.image;
end
image_gray = histeq(image_gray);
image_gauss = imgaussfilt(image_gray, 1.0);
[img_sobel, ~] = imgradient(image_gauss, 'sobel');
data_prep.img_sobel = img_sobel / max(max(img_sobel));

% downsample sobel image
[m, n] = size(img_sobel);
mask(1:m, 1:n) = 1;
mask(1:2:m, 1:2:n) = 0;
data_prep.img_sobel_dw = data_prep.img_sobel .* mask; %edge(image_gray, 'canny', 0.05);

% preprocess lidar data
scan_filtered = filterScanAzimuth(data.scan, params);
[image_depth, image_discnt, image_world] = imageDepthFromLidar(scan_filtered, data, params);
data_prep.img_depth = image_depth;
data_prep.img_discnt = image_discnt;
data_prep.image_world = image_world;
    
end