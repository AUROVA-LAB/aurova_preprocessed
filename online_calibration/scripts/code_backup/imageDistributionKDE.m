function kde = imageDistributionKDE(image, w, min, max)

% variable declarations
kde(min:max) = double(0);
[v, u, c] = size(image);

for n = min:max
    for i = min:max
        K = n - i;
        k = kernelGaussian(K, w);
        N = length(image(image == i));
        kde(n) = kde(n) + k*N;
    end
    kde(n) = kde(n) / (u*v);
end

end

