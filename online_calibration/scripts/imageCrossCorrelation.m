function cc_map = imageCrossCorrelation(image, template, x, y, W, H)

% variable declarations
[v, u, c] = size(image);
[h, w, k] = size(template);
cc_map(1:v, 1:u) = double(0);
image = double(image);
template = double(template);
% template = template - mean(mean(template));

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

% correlation in all image
for i = ini_x:end_x
    for j =  ini_y:end_y 
        roi = image(j:j+h-1, i:i+w-1);
%         roi = roi - mean(mean(roi)); % to prevent uniform roi
        cc_map(j, i) = sum(sum(template .* roi));
    end
end

% normalization
cc_map = cc_map / max(max(cc_map));

end

