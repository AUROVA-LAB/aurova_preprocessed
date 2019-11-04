function kl_divergence = imageKLDivergence(image, template, x, y, W, H)

% variables declaration
[v, u, c] = size(image);
[h, w, k] = size(template);
kl_divergence(1:v, 1:u) = double(0);
template = template +1; % +1 to prevent 0 index (and regularization for 2D case)

% definition of macro-roi
ini_x = x - W;
end_x = x + W;
ini_y = y - H;
end_y = y + H;
if ini_x < 1
    ini_x = 1;
end
if ini_y < 1
    ini_y = 1;
end
if end_x > u-w
    end_x = u-w;
end
if end_y > v-h
    end_y = v-h;
end

% KL divergence calculation (2D distribution)
for i = ini_x:end_x
    for j =  ini_y:end_y 
        roi = double(image(j:j+h-1, i:i+w-1)) + double(1); % regularization
        roi = roi / sum(sum(roi));
        template = double(template);
        kl_divergence(j, i) = kl_divergence(j, i) + sum(sum(template .* log2(template)));
        kl_divergence(j, i) = kl_divergence(j, i) - sum(sum(template .* log2(roi)));
    end
end

% % KL divergence calculation (1D distribution)
% template_dist  = imageDistributionMLE(template, MIN_PIXEL, MAX_PIXEL);
% for i = 1:u-w
%     for j = 1:v-h
%         
%         roi = image(j:j+h-1, i:i+w-1) + 1; % +1 to prevent 0 index
%         roi_dist = imageDistributionMLE(roi, MIN_PIXEL, MAX_PIXEL);
%         kl_divergence(j, i) = kl_divergence(j, i) + sum(template_dist .* log2(template_dist));
%         kl_divergence(j, i) = kl_divergence(j, i) - sum(template_dist .* log2(roi_dist)); 
%     end
% end

end

