function data_prep = prerpocessData(data, params)

data_prep = [];

% preprocess camera data image
image_gray = rgb2gray(data.image);
[img_sobel, ~] = imgradient(image_gray, 'sobel');
data_prep.img_sobel = img_sobel;

% preprocess lidar data
scan_filtered = filterScanAzimuth(data.scan, params);
[image_depth, image_discnt] = imageDepthFromLidar(scan_filtered, data, params);
data_prep.img_depth = image_depth;
data_prep.img_discnt = image_discnt;
    
end