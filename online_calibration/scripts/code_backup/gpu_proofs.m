clear, clc, close all

N = 400;
source_x = 1:N;
source_y = 1:N;
source = cat(2, source_x', source_y');
%source = gpuArray(source);
i(1:N*N) = 0;
j(1:N*N) = 0;

t = tic;

% for u = 1:N
%     for v = 1:N
%         ii = (u-1)*v + v;
%         i(ii) = floor((ii-1) / N) + 1;
%         j(ii) = mod(ii-1, N) + 1;
%         source_x_1 = source_x(u);
%         source_x_2 = source_x(v);
%         source_y_1 = source_y(u);
%         source_y_2 = source_y(v);
%         [src0_x, src0_y] = gpuProofs(source_x_1);
%     end
% end

time_orig = toc(t)

% parfor ii = 1:N*N
%     i(ii) = floor((ii-1) / N) + 1;
%     j(ii) = mod(ii-1, N) + 1;
%     source_x_1 = source_x(i(ii));
%     source_x_2 = source_x(j(ii));
%     source_y_1 = source_y(i(ii));
%     source_y_2 = source_y(j(ii));
%     [src0_x, src0_y] = gpuProofs(source_x_1, source_x_2, source_y_1, source_y_2);
% end

time_vec = toc(t) - time_orig

ii = gpuArray(1:N*N);
[i, j] = arrayfun(@gpuProofs2, ii, N);
i(1) = 1;

time_gpu = toc(t) - time_vec - time_orig
