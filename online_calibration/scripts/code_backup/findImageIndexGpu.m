function index = findImageIndexGpu(x_tmp, y_tmp, x_kp_src, y_kp_src, x_pair_src, y_pair_src, ...
                                dist_tmp, rot_tmp, w, h)

dist_src = sqrt(((x_pair_src-x_kp_src) .* (x_pair_src-x_kp_src)) + ...
                ((y_pair_src-y_kp_src) .* (y_pair_src-y_kp_src)));
rot_src = atan2((y_pair_src-y_kp_src), (x_pair_src-x_kp_src)) * 180.0 / pi;

% scale TEMPLATE points to SOURCE scale
scale_factor = dist_src / dist_tmp;
x_tmp_act = x_tmp .* scale_factor;
y_tmp_act = y_tmp .* scale_factor;

% translate and rorate TEMPLATE points to SOURCE pose
yaw = (rot_src - rot_tmp) .* (pi/180);
tr_x_tmp = round(x_tmp_act.*cos(yaw) + y_tmp_act.*sin(yaw) + x_kp_src);
tr_y_tmp = round(y_tmp_act.*cos(yaw) - x_tmp_act.*sin(yaw) + y_kp_src);

% if tr_x_tmp >= 1 && tr_x_tmp <= w && ...
%    tr_y_tmp >= 1 && tr_y_tmp <= h
    u = tr_x_tmp;
    v = tr_y_tmp;
% else
%     u = w;
%     v = h;
% end

index = (u-1).*h + v;

end

