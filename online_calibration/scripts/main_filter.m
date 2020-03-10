clear, clc, close all

load('results/obs_20200310.mat')
load('results/gt_20200310.mat')
load('noise_model.mat')
num_samples = length(ext_obs);

state = initState();

x = [];
gt_x(1:num_samples) = gt(1);
y = [];
gt_y(1:num_samples) = gt(2);
z = [];
gt_z(1:num_samples) = gt(3);
w = [];
gt_w(1:num_samples) = gt(4);
p = [];
gt_p(1:num_samples) = gt(5);
r = [];
gt_r(1:num_samples) = gt(6);
for i = 1:num_samples
    observation = setObservation(ext_obs{i}, noise_model);
    state = updateState(observation, state);
    x = cat(2, x, state.stt(1));
    y = cat(2, y, state.stt(2));
    z = cat(2, z, state.stt(3));
    w = cat(2, w, state.stt(4));
    p = cat(2, p, state.stt(5));
    r = cat(2, r, state.stt(6));
end

lim = 138;
subplot(6, 1, 1)
plot(x(1:lim), 'LineWidth', 2)
hold on
plot(gt_x(1:lim), 'LineWidth', 2)
subplot(6, 1, 2)
plot(y(1:lim), 'LineWidth', 2)
hold on
plot(gt_y(1:lim), 'LineWidth', 2)
subplot(6, 1, 3)
plot(z(1:lim), 'LineWidth', 2)
hold on
plot(gt_z(1:lim), 'LineWidth', 2)
subplot(6, 1, 4)
plot(w(1:lim), 'LineWidth', 2)
hold on
plot(gt_w(1:lim), 'LineWidth', 2)
subplot(6, 1, 5)
plot(p(1:lim), 'LineWidth', 2)
hold on
plot(gt_p(1:lim), 'LineWidth', 2)
subplot(6, 1, 6)
plot(r(1:lim), 'LineWidth', 2)
hold on
plot(gt_r(1:lim), 'LineWidth', 2)