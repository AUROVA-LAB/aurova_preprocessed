function data = preprocessData(data, params)

% preprocess camera data image
[~, ~, c] = size(data.input.image);
if c == 3
    image_gray = rgb2gray(data.input.image);
else
    image_gray = data.input.image;
end
image_gray = histeq(image_gray);
image_gauss = imgaussfilt(image_gray, 1.0);
[img_sobel, ~] = imgradient(image_gauss, 'sobel');
data.process.img_sobel = img_sobel / max(max(img_sobel));

% downsample sobel image
[m, n] = size(img_sobel);
mask(1:m, 1:n) = 1;
mask(1:2:m, 1:2:n) = 0;
data.process.img_sobel_src = data.process.img_sobel;

% preprocess lidar data
data.process.scan_filtered = filterScanAzimuth(data.input.scan, params);
data = imageDepthFromLidar(data, params);
    
end