clear, clc, close all

M = 1403;
N = 1673;
N2 = 1665;

tmp_x(1:M) = 1;
tmp_y(1:M) = 2;
kp_x(1:N) = 3;
kp_y(1:N) = 4;
pair_x(1:N2) = 5;
pair_y(1:N2) = 6;

t = tic;

% for ii = 1:N*N2
%     id_kp = floor((ii-1) / N2) + 1;
%     id_pair = mod(ii-1, N2) + 1;
%     for jj = 1:M
%         [a, b] = gpuProofs(kp_x(id_kp), kp_y(id_kp), ...
%                            pair_x(id_pair), pair_y(id_pair), ...
%                            tmp_x(jj), tmp_y(jj));
%     end
% end

time_orig = toc(t)

tmp_x(1:M*N2) = 1;
tmp_y(1:M*N2) = 2;
pair_x(1:M*N2) = 5;
pair_y(1:M*N2) = 6;
tmp_x = gpuArray(tmp_x);
tmp_y = gpuArray(tmp_y);
pair_x = gpuArray(pair_x);
pair_y = gpuArray(pair_y);

escalar1 = 5;
escalar2 = 6;

t = tic;

for ii = 1:N
    [a, b] = gpuProofs(kp_x(ii), kp_y(ii), ...
                       pair_x, pair_y, ...
                       tmp_x, tmp_y, escalar1, escalar2);
end

time_gpu = toc(t)
