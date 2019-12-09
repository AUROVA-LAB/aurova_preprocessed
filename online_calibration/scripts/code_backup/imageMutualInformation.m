function mutual_info = imageMutualInformation(image, template)

% constants declarations
MIN_PIXEL = 1;
MAX_PIXEL = 256;

% variables declaration
template = template + 1; % +1 to prevent 0 index
image = image +1; % +1 to prevent 0 index
template = double(template);
image = double(image);
[v, u, c] = size(image);
[h, w, k] = size(template);
mutual_info(1:v, 1:u) = double(0);

% Mutual information (for 1D distribution)
template_dist = imageDistributionMLE(template, MIN_PIXEL, MAX_PIXEL);
for i = 1:u-w
    for j = 1:v-h
        
        roi = image(j:j+h-1, i:i+w-1); 
        roi_dist = imageDistributionMLE(roi, MIN_PIXEL, MAX_PIXEL);
        join_dist = template_dist' * roi_dist;
        
       mutual_info(j, i) = mutual_info(j, i) - sum(template_dist .* log2(template_dist));
       mutual_info(j, i) = mutual_info(j, i) - sum(roi_dist .* log2(roi_dist));
       mutual_info(j, i) = mutual_info(j, i) + sum(sum(join_dist .* log2(join_dist)));
    
    end
end

end

