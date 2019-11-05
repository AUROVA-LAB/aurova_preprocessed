function kde = imageJoinKDE(image_1, image_2, min, max)

% variable declarations
kde(min:max, min:max) = double(0);
[v, u, c] = size(image_1);

% covariance matrix
% TODO: calculate (s1, s2) from data
s1 = 3;
s2 = 4;
W = [s1 0; 0 s2];
W = double(W);

% KDE calculation
for n1 = min:max
    for n2 = min:max
%         K1(1:v, 1:u) = n1 - image_1;
%         K2(1:v, 1:u) = n2 - image_2;
%         K(1:2, 1:v, 1:u) = double(0);
%         K(1, :, :) = K1;
%         K(2, :, :) = K2;
%         C = num2cell(K, 1);
%         C_out = cellfun(@kernelGaussian,C,'UniformOutput',0);
%         A = cell2mat(C_out);
%         k = sum(sum(A));
%         kde(n1, n2) = k  / (u*v);
        for i = 1:u
            for j = 1:v
                K1 = n1 - image_1(j, i);
                K2 = n2 - image_2(j, i);
                K = [K1; K2];
                K = double(K);
                k = kernelGaussian(K, W);
                kde(n1, n2) = kde(n1, n2) + k;
            end
        end
    end
end

end

