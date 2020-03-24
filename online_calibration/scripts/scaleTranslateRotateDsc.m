function [tr_x_tmp, tr_y_tmp] = scaleTranslateRotateDsc(data, i)

kp_s = data.matches.current.kp_src(i, :);
pair_s = data.matches.current.pair_src(i, :);
[dist_s, rot_s, ~] = cartesian2SphericalInDegrees(pair_s(1) - kp_s(1), pair_s(2) - kp_s(2), 0);

kp_t = data.matches.current.kp_tmp(i, :);
dist_t = data.matches.current.descriptor.distance(i);
rot_t = data.matches.current.descriptor.rotation(i);

x_tmp = data.matches.current.descriptor.cluster{i}(:, 1);
y_tmp = data.matches.current.descriptor.cluster{i}(:, 2);
x_tmp = x_tmp - kp_t(1); % tr to kp reference
y_tmp = y_tmp - kp_t(2);

% scale TEMPLATE points to SOURCE scale
scale_factor = dist_s / dist_t;
x_tmp_act = x_tmp * scale_factor;
y_tmp_act = y_tmp * scale_factor;

% translate and rorate TEMPLATE points to SOURCE pose
yaw = (rot_s - rot_t) * (pi/180);
tr_x_tmp = x_tmp_act*cos(yaw) + y_tmp_act*sin(yaw) + kp_s(1);
tr_y_tmp = y_tmp_act*cos(yaw) - x_tmp_act*sin(yaw) + kp_s(2);

end

