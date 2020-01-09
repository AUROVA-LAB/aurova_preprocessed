function dilated_images = imageDilate(images, n, t)

[h, w, c] = size(images);
dilated_images = images;

for i = 1:w-n
    for j = 1:h-n
        for k = 1:c
            roi = images(j:j+n-1, i:i+n-1, k);
            if sum(sum(roi)) > t
                dilated_images(j+(n-1)/2, i+(n-1)/2, k) = 1;
            end
        end
    end
end

end

