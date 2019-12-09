function mle = imageDistributionMLE(image, min, max)

% variable declarations
mle(min:max) = double(1); % '1' to prevent '0' for entropies
[v, u, c] = size(image);

% histogram of image
for i=1:u
    for j=1:v
        mle(image(j, i)) = mle(image(j, i)) + 1;
    end
end

% normalization
mle = mle / sum(mle);

end

