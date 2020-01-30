function plot_info = findImageKpCorrespondence(data_prep, descriptor, params)


plot_info = [];
[num_kp, ~] = size(descriptor.kp);
plot_info.kp_src(1:num_kp, 1:2) = 0;
plot_info.kp_tmp(1:num_kp, 1:2) = 0;

for j = 1:num_kp

    % generate TEMPLATE xyz data
    threshold = params.threshold_dsc;
    [y_tmp, x_tmp] = find(data_prep.img_discnt > threshold); % cloud template
    z_tmp = data_prep.img_discnt(data_prep.img_discnt > threshold);
    z_tmp = double(z_tmp) / params.base;
    x_tmp = x_tmp - descriptor.kp(j, 1); % tr to kp reference
    y_tmp = y_tmp - descriptor.kp(j, 2);

    % generate SOURCE image data 
    source = data_prep.img_sobel;

    % generate sobel-KEYPOINTs
    threshold = params.threshold_sbl;
    clear source_msk;
    source_msk = source(descriptor.roi.p11(j, 2):descriptor.roi.p21(j, 2), ...
                        descriptor.roi.p11(j, 1):descriptor.roi.p12(j, 1));
    [kp_y, kp_x] = find(source_msk > threshold);
    kp_y = kp_y + descriptor.roi.p11(j, 2); %translate to image coord.
    kp_x = kp_x + descriptor.roi.p11(j, 1);

    % find sobel-KEYPOINTs that maximize cost function
    N = length(kp_y);
    clear vector;
    vector(1:N*N) = double(0);
    dist_tmplt = descriptor.distance(j);
    rot_tmplt = descriptor.rotation(j);
    w = params.camera_params.image_size(2);
    h = params.camera_params.image_size(1);
    dist_min = dist_tmplt * params.dist_factor;
    dist_max = dist_tmplt / params.dist_factor;
    disp('*** init pair search ***')
    t = tic;
    parfor ii = 1:N*N 
        id_kp = floor((ii-1) / N) + 1;
        id_pair = mod(ii-1, N) + 1;
        [dist_src, rot_src, ~] = cartesian2SphericalInDegrees(kp_x(id_pair) - kp_x(id_kp), kp_y(id_pair) - kp_y(id_kp), 0);
        if dist_src > dist_min && dist_src < dist_max

            % scale TEMPLATE points to SOURCE scale
            scale_factor = dist_src / dist_tmplt;
            x_tmp_act = x_tmp * scale_factor;
            y_tmp_act = y_tmp * scale_factor;

            % translate and rorate TEMPLATE points to SOURCE pose
            yaw = (rot_src - rot_tmplt) * (pi/180);
            tr_x_tmp = x_tmp_act*cos(yaw) + y_tmp_act*sin(yaw) + kp_x(id_kp);
            tr_y_tmp = y_tmp_act*cos(yaw) - x_tmp_act*sin(yaw) + kp_y(id_kp);

            % evaluate COST FUNCTION in pair id_kp id_pair using TEMPLATE and SOURCE
            for i = 1:length(x_tmp_act)
                if tr_x_tmp(i) >= 1 && tr_x_tmp(i) <= w && ...
                   tr_y_tmp(i) >= 1 && tr_y_tmp(i) <= h
                    u = round(tr_x_tmp(i));
                    v = round(tr_y_tmp(i));
                    vector(ii) = vector(ii) + source(v, u) * z_tmp(i);
                end
            end 
        end
    end
    disp(toc(t))

    %maximun in cost function
    ii = find(vector==max(max(vector)));
    id_kp = floor((ii-1) / N) + 1;
    plot_info.kp_src(j, 1) = kp_x(id_kp);
    plot_info.kp_src(j, 2) = kp_y(id_kp);
    plot_info.kp_tmp(j, 1) = descriptor.kp(j, 1);
    plot_info.kp_tmp(j, 2) = descriptor.kp(j, 2);

end

end