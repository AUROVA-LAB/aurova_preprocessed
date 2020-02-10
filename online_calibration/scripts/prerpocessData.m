function data_prep = prerpocessData(data, params)

data_prep = [];

% preprocess camera data image
image_gray = rgb2gray(data.image);
[img_sobel, ~] = imgradient(image_gray, 'sobel');
data_prep.img_sobel = img_sobel;

% downsample sobel image
[m, n] = size(img_sobel);
mask(1:m, 1:n) = 0;
mask(1:2:m, 1:2:n) = 1;
data_prep.img_sobel_dw = img_sobel .* mask;

% preprocess lidar data
scan_filtered = filterScanAzimuth(data.scan, params);
[image_depth, image_discnt] = imageDepthFromLidar(scan_filtered, data, params);
data_prep.img_depth = image_depth;
data_prep.img_discnt = image_discnt;
    
end