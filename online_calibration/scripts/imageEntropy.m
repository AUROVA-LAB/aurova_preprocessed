function [entropy, entropy_mask] = imageEntropy(image, w, h)

image = double(image);
[v, u, k] = size(image);
entropy(1:v, 1:u) = double(0);
entropy_mask =  zeros(v, u);


% maximum entropy considered
% roi(1:h, 1:w) = double(1);
% roi = roi / sum(sum(roi));
% entropy_max = -sum(sum(roi .* log2(roi)));
% 2D distribution
% for i = 1:u-w
%     for j = 1:v-h
%         
%         roi = image(j:j+h-1, i:i+w-1); 
%         roi = roi / sum(sum(roi));
%         
%         for m = 1:h
%             for n = 1:w
%                 if roi(m, n) > 0
%                     entropy(j, i) = entropy(j, i) - roi(m, n) .* log2(roi(m, n));
%                 end
%             end
%         end
% 
%         if (entropy(j, i) < entropy_max * 0.75) % TODO: adjustable
%             entropy_mask(j, i) = 0;
%         else
%             entropy_mask(j, i) = 1;
%         end
%     end
% end

% 1D distribution
hist(1:256) = double(1);
hist = hist / sum(hist);
entropy_max = -sum(hist .* log2(hist));
for i = 1:u-w
    for j = 1:v-h
        
        roi = image(j:j+h-1, i:i+w-1) + 1; % +1 to prevent 0 index
        hist = normalizedHistogramMLE(roi, 1, 256);
        
        for n = 1:length(hist)
            if (hist(n) > 0)
                entropy(j, i) = entropy(j, i) - hist(n) .* log2(hist(n));
            end
        end
        
        if (entropy(j, i) < entropy_max * 0.5) % TODO: adjustable
            entropy_mask(j, i) = 0;
        else
            entropy_mask(j, i) = 1;
        end
        
    end
end

entropy = entropy / entropy_max;

end

