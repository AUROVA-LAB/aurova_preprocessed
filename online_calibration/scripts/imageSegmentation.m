function [objects_gray, objects_depth] = imageSegmentation(image, image_depth, index, read_mat)

if read_mat
    objects_gray = load(num2str(index,'mat_data/objects_gray%d.mat'), 'objects_gray');
    objects_depth = load(num2str(index,'mat_data/objects_depth%d.mat'), 'objects_depth');
    objects_gray = objects_gray.objects_gray;
    objects_depth = objects_depth.objects_depth;
else
    MIN_BLOB_AREA = 2000;
    [h, w, c] = size(image_depth);

    %*********** preprocess images ************%
    image_gray = rgb2gray(image);
    %image_gray = imgaussfilt(image_gray, 2.0);
    [image_grad, image_dir] = imgradient(image_gray, 'sobel');

    %********* segmentation of depth im ********%
    k = 4;
    if c == 1
        image_depth_3c = cat(3, image_depth, image_depth, image_depth);
    else
        image_depth_3c = image_depth;
    end
    image_depth_seg = imageKmeansSeg(image_depth_3c, k);
    image_depth_slices = zeros(h, w, k);
    for n = 1:k
        image_depth_slices(:, :, n) = image_depth_seg == n;
    end
    d = 4;
    image_depth_dilated = image_depth_slices;
    for n = 1:d
        image_depth_dilated = imageDilate(image_depth_dilated, 3, 2);
    end
    e = 4;
    for n = 1:e
        image_depth_dilated = imageErode(image_depth_dilated, 3, 2);
    end

    %************ blobs extraction *************%
    st_blob = struct('image_blob',	[], ...
                              'label',            0, ...
                              'centroid',       [], ...
                              'area',             0, ...
                              'bbox',            [], ...
                              'cluster_id',     0);
    hblob = vision.BlobAnalysis;
    hblob.release();
    hblob.LabelMatrixOutputPort  = true;
    blob = st_blob; 
    blobs_per_seg = []; 
    for n = 1:k
        [area, centroid, bbox, labels] = step(hblob, logical(image_depth_dilated(:, :, n)));
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
    objects_gray = [];
    objects_depth = [];
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
        %mask = activecontour(image, mask);

        %******** generate images for plot **********%
        %objects_gray = cat(3, objects_gray, image_gray .* uint8(mask));
        objects_gray = cat(3, objects_gray, uint8(image_grad) .* uint8(mask));
        objects_depth = cat(3, objects_depth, uint8(blobs_per_seg(n).image_blob) .* uint8(image_depth_slices(:, :, kn)) * 255);
    end

    save(num2str(index,'mat_data/objects_gray%d.mat'), 'objects_gray');
    save(num2str(index,'mat_data/objects_depth%d.mat'), 'objects_depth');
end

end

