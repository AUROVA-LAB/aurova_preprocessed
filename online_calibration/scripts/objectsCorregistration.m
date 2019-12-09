function plot_gray = objectsCorregistration(objects_gray, objects_depth, index)

MIN_BLOB_AREA = 50;
THRESHOLD_TM = 20;
THRESHOLD = 20;
[h, w, c] = size(objects_gray);

image_filename_base = 'raw_data/input/raw_data_01/image';
image_filename = strcat(image_filename_base, num2str(index,'%d.jpg'));
image = imread(image_filename);
    
image_gray = rgb2gray(image);
plot_gray_r = image_gray;
plot_gray_g = image_gray;
plot_gray_b = image_gray;
plot_gray = zeros(h, w, 3, 'uint8');


for n = 1:c
    
    if n ~= 3 % ground
        gray = objects_gray(:, :, n);
        depth = objects_depth(:, :, n);

        %************* edges detection *************%
        st_blob = struct('image_blob',	[], ...
                                  'label',            0, ...
                                  'centroid',       [], ...
                                  'area',             0, ...
                                  'bbox',            [], ...
                                  'cluster_id',     0);
        canny = edge(gray, 'canny', 0.2);
        canny = uint8(canny) * 255;
        depth_dil = imageDilate(depth > 20, 3, 3);
        depth_dil = imageDilate(depth_dil, 3, 3);
        depth_dil = imageDilate(depth_dil, 3, 3);
        hblob = vision.BlobAnalysis;
        hblob.release();
        hblob.LabelMatrixOutputPort  = true;
        [area, centroid, bbox, labels] = step(hblob, depth_dil);
        depth_filtered = zeros(h, w, 1, 'logical');
        for m = 1:length(area)
            if area(m) > MIN_BLOB_AREA
                depth_filtered = depth_filtered + (labels == m);
            end
        end
        depth_edges = imgradient(double(depth_filtered)*255, 'sobel');

        %***************** ICP ********************%
        [pt_y_tm, pt_x_tm] = find(depth_edges > THRESHOLD_TM); % cloud template
        clear pt_z_tm;
        pt_z_tm(1:length(pt_x_tm), 1) = double(0);
        pt_xyz_tm = [pt_x_tm'; pt_y_tm'; pt_z_tm']';
        pt_cloud_tm = pointCloud(pt_xyz_tm);
        [pt_y, pt_x] = find(canny > THRESHOLD); % cloud image
        clear pt_z;
        pt_z(1:length(pt_x), 1) = double(0);
        pt_xyz = [pt_x'; pt_y'; pt_z']';
        pt_cloud = pointCloud(pt_xyz);
        t_form = pcregistericp(pt_cloud_tm, pt_cloud); % register ICP
        pt_cloud_match = pctransform(pt_cloud_tm, t_form);

        %*****************************************%
        plot_gray_r(depth_edges > 20) = 255;
        plot_gray_g(depth_edges > 20) = 0;
        plot_gray_b(depth_edges > 20) = 0;

        for m = 1:pt_cloud_match.Count
            u = round(pt_cloud_match.Location(m, 1));
            v = round(pt_cloud_match.Location(m, 2));
            if u > 0 && u <= w && v > 0 && v <= h
                plot_gray_r(v, u) = 0;
                plot_gray_g(v, u) = 255;
                plot_gray_b(v, u) = 0;
            end
        end
        plot_gray(:, :, 1) = plot_gray_r;
        plot_gray(:, :, 2) = plot_gray_g;
        plot_gray(:, :, 3) = plot_gray_b;
    end
end

end

