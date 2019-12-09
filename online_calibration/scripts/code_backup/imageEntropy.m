function [entropy, entropy_mask] = imageEntropy(image, w, h)

% variable declarations
image = double(image);
[v, u, k] = size(image);
entropy(1:v, 1:u) = double(0);
entropy_mask =  zeros(v, u);

% entropy using rois (for 1D distribution)
template_dist(1:256) = double(1);
template_dist = template_dist / sum(template_dist);
entropy_max = -sum(template_dist .* log2(template_dist));
for i = 1:u-w
    for j = 1:v-h
        
        roi = image(j:j+h-1, i:i+w-1) + 1; % +1 to prevent 0 index
        template_dist = imageDistributionMLE(roi, 1, 256);
        entropy(j, i) = entropy(j, i) - sum(template_dist .* log2(template_dist));
        
        if (entropy(j, i) < entropy_max * 0.77) % TODO: adjustable
            entropy_mask(j, i) = 0;
        else
            entropy_mask(j, i) = 1;
        end
    end
end

% % entropy using rois (for 2D distribution)
% roi(1:h, 1:w) = double(1);
% roi = roi / sum(sum(roi));
% entropy_max = -sum(sum(roi .* log2(roi)));
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

end

