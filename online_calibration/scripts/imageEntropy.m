function [entropy, entropy_mask] = imageEntropy(image, w, h)

image = double(image);
image = image + 1.0;
[v, u, k] = size(image);
entropy = zeros(v, u);
entropy_mask =  zeros(v, u);


% maximum entropy considered
roi(1:h, 1:w) = double(100); % TODO: adjustable
entropy_max = sum(sum(roi .* log2(roi)));

% 2D distribution
for i = 1:u-w
    for j = 1:v-h
        roi = image(j:j+h-1, i:i+w-1); 
        entropy(j, i) = sum(sum(roi .* log2(roi)));
        if (entropy(j, i) < entropy_max / 4) % TODO: adjustable
            entropy_mask(j, i) = 0;
        else
            entropy_mask(j, i) = 1;
        end
    end
end

% 1D distribution
% for i = 1:u-w
%     for j = 1:v-h
%         mask = image(j:j+h-1, i:i+w-1) + 1;
%         h = histogramLocal(mask, 1, 256 + 1);
%         entropy(j, i) = sum(h .* log2(h));
%     end
% end

entropy = entropy / entropy_max;

end

