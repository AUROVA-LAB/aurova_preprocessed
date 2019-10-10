clear, clc, close all

%************** read images ***************%
filename_sobel_base = 'images/tr_01/sobel';
filename_edges_base = 'images/tr_01/edges';
index = 150;
filename_sobel = strcat(filename_sobel_base, num2str(index,'%d'));
filename_edges = strcat(filename_edges_base, num2str(index,'%d'));
filename_sobel = strcat(filename_sobel, '.jpg');
filename_edges = strcat(filename_edges, '.jpg');
image_sobel = imread(filename_sobel);
image_edges = imread(filename_edges);


%*********** representation ***************%
figure %subplot(2, 1, 1);
imshow(image_sobel);
figure %subplot(2, 1, 2);
imshow(image_edges);