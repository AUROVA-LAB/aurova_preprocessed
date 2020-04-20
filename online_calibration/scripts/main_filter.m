clear, clc, close all

load('results/obs_20200226.mat')
load('results/gt_20200226.mat')
load('noise_model.mat')
num_samples = length(ext_obs);
lim = 100;
set_ini = 126;
set_end = set_ini + lim - 1;


[tf_err, state, lk] = generateMisscalibration();

x = [];
x_o = [];
ex = [];
ex_o = [];
gt_x(1:num_samples) = gt(1);
y = [];
y_o = [];
ey = [];
ey_o = [];
gt_y(1:num_samples) = gt(2);
z = [];
z_o = [];
ez = [];
ez_o = [];
gt_z(1:num_samples) = gt(3);
w = [];
w_o = [];
ew = [];
ew_o = [];
gt_w(1:num_samples) = gt(4);
p = [];
p_o = [];
ep = [];
ep_o = [];
gt_p(1:num_samples) = gt(5);
r = [];
r_o = [];
er = [];
er_o = [];
gt_r(1:num_samples) = gt(6);

x_o = cat(2, x_o, 0.0);
y_o = cat(2, y_o, 0.0);
z_o = cat(2, z_o, 0.0);
w_o = cat(2, w_o, 0.0);
p_o = cat(2, p_o, 0.0);
r_o = cat(2, r_o, 0.0);
ex_o = cat(2, ex_o, 0.0);
ey_o = cat(2, ey_o, 0.0);
ez_o = cat(2, ez_o, 0.0);
ew_o = cat(2, ew_o, 0.0);
ep_o = cat(2, ep_o, 0.0);
er_o = cat(2, er_o, 0.0);
x = cat(2, x, state.stt(1));
y = cat(2, y, state.stt(2));
z = cat(2, z, state.stt(3));
w = cat(2, w, state.stt(4));
p = cat(2, p, state.stt(5));
r = cat(2, r, state.stt(6));
ex = cat(2, ex, state.cov(1, 1));
ey = cat(2, ey, state.cov(2, 2));
ez = cat(2, ez, state.cov(3, 3));
ew = cat(2, ew, state.cov(4, 4));
ep = cat(2, ep, state.cov(5, 5));
er = cat(2, er, state.cov(6, 6));

for i = set_ini:set_end
    observation = setObservation(ext_obs{i}, noise_model);
    state = updateState(observation, state);
    x_o = cat(2, x_o, observation.obs(1));
    y_o = cat(2, y_o, observation.obs(2));
    z_o = cat(2, z_o, observation.obs(3));
    w_o = cat(2, w_o, observation.obs(4));
    p_o = cat(2, p_o, observation.obs(5));
    r_o = cat(2, r_o, observation.obs(6));
    ex_o = cat(2, ex_o, observation.cov(1, 1));
    ey_o = cat(2, ey_o, observation.cov(2, 2));
    ez_o = cat(2, ez_o, observation.cov(3, 3));
    ew_o = cat(2, ew_o, observation.cov(4, 4)^2);
    ep_o = cat(2, ep_o, observation.cov(5, 5)^2);
    er_o = cat(2, er_o, observation.cov(6, 6)^2);
    x = cat(2, x, state.stt(1));
    y = cat(2, y, state.stt(2));
    z = cat(2, z, state.stt(3));
    w = cat(2, w, state.stt(4));
    p = cat(2, p, state.stt(5));
    r = cat(2, r, state.stt(6));
    ex = cat(2, ex, state.cov(1, 1));
    ey = cat(2, ey, state.cov(2, 2));
    ez = cat(2, ez, state.cov(3, 3));
    ew = cat(2, ew, state.cov(4, 4));
    ep = cat(2, ep, state.cov(5, 5));
    er = cat(2, er, state.cov(6, 6));
end

% STATE EVOLUTION PLOT
samples = 1:lim;
figure
subplot(2, 3, 1)
plot(x(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('x (m)')
hold on
plot(x(1:lim) + sqrt(ex(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(x(1:lim) - sqrt(ex(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_x(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 2)
plot(y(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('y (m)')
hold on
plot(y(1:lim) + sqrt(ey(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(y(1:lim) - sqrt(ey(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_y(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 3)
plot(z(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('z (m)')
hold on
plot(z(1:lim) + sqrt(ez(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(z(1:lim) - sqrt(ez(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_z(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 4)
plot(w(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('yaw (dgr)')
hold on
plot(w(1:lim) + sqrt(ew(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(w(1:lim) - sqrt(ew(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_w(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 5)
plot(p(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('pitch (dgr)')
hold on
plot(p(1:lim) + sqrt(ep(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(p(1:lim) - sqrt(ep(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_p(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 6)
plot(r(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('roll (dgr)')
hold on
plot(r(1:lim) + sqrt(er(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(r(1:lim) - sqrt(er(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_r(1:lim),  'r', 'LineWidth', 1)
grid

% OBSERVETION PLOT
figure
subplot(2, 3, 1)
plot(x_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('x (m)')
hold on
plot(x_o(1:lim) + ex_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(x_o(1:lim) - ex_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_x(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 2)
plot(y_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('y (m)')
hold on
plot(y_o(1:lim) + ey_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(y_o(1:lim) - ey_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_y(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 3)
plot(z_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.5 0.5])
xlabel('Samples')
ylabel('z (m)')
hold on
plot(z_o(1:lim) + ez_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(z_o(1:lim) - ez_o(1:lim), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_z(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 4)
plot(w_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('yaw (dgr)')
hold on
plot(w_o(1:lim) + sqrt(ew_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(w_o(1:lim) - sqrt(ew_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_w(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 5)
plot(p_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('pitch (dgr)')
hold on
plot(p_o(1:lim) + sqrt(ep_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(p_o(1:lim) - sqrt(ep_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_p(1:lim),  'r', 'LineWidth', 1)
grid
subplot(2, 3, 6)
plot(r_o(1:lim), 'LineWidth', 2)
axis([1 lim -0.8 0.8])
xlabel('Samples')
ylabel('roll (dgr)')
hold on
plot(r_o(1:lim) + sqrt(er_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(r_o(1:lim) - sqrt(er_o(1:lim)), '--', 'color', [0.5 0.5 0.5], 'LineWidth', 1)
plot(gt_r(1:lim),  'r', 'LineWidth', 1)
grid

% ERROR CALCULATION
e = x(lim/2:end);
error_x = mean(e)*100
std_x = std(e)*100
e = y(lim/2:end);
error_y = mean(e)*100
std_y = std(e)*100
e = z(lim/2:end);
error_z = mean(e)*100
std_z = std(e)*100
e = w(lim/2:end);
error_w = mean(e)*100
std_w = std(e)*100
e = p(lim/2:end);
error_p = mean(e)*100
std_p = std(e)*100
e = r(lim/2:end);
error_r = mean(e)*100
std_r = std(e)*100