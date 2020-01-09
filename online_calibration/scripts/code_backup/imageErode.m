function eroded_images = imageErode(images, n, t)

[h, w, c] = size(images);
eroded_images = images;

for i = 1:w-n
    for j = 1:h-n
        for k = 1:c
            roi = images(j:j+n-1, i:i+n-1, k);
            if sum(sum(roi)) < (n*n - t)
                eroded_images(j+(n-1)/2, i+(n-1)/2, k) = 0;
            end
        end
    end
end

end

