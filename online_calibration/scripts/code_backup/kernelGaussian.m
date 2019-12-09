function k = kernelGaussian(x, w)

D = length(x);
k = 1 / ((2*pi)^(D/2) * norm(w)^(1/2)) * exp(- (1/2) * x' * inv(w) * x);

end

