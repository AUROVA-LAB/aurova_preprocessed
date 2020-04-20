function data = preprocessData(data, params)

% preprocess camera data image
[~, ~, c] = size(data.input.image);
if c == 3
    image_gray = rgb2gray(data.input.image);
else
    image_gray = data.input.image;
end
image_hist = histeq(image_gray);
image_gauss = imgaussfilt(image_hist, 1.0);
[img_sobel, ~] = imgradient(image_gauss, 'sobel');
data.process.img_sobel = img_sobel; %imgaussfilt(img_sobel, 1.0);
data.process.img_sobel = data.process.img_sobel / max(max(data.process.img_sobel));
data.process.img_gauss = image_gauss;
data.process.img_hist = image_hist;

% canny
image_canny = edge(image_gray, 'canny', 0.1);
data.process.img_canny = imgaussfilt(double(image_canny), 1.0);

% generate sourde image
data.process.img_sobel_src = data.process.img_sobel;

% preprocess lidar data
data.process.scan_filtered = filterScanAzimuth(data.input.scan, params);
data = imageDepthFromLidar(data, params);
    
end