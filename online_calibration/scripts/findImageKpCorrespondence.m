function matches = findImageKpCorrespondence(data_prep, descriptors, params)

matches = [];
[num_kp, ~] = size(descriptors.kp);
matches.kp_src(1:num_kp, 1:2) = 1;
matches.kp_tmp(1:num_kp, 1:2) = 1;
matches.pair_src(1:num_kp, 1:2) = 1;
matches.pair_tmp(1:num_kp, 1:2) = 1;
matches.descriptor = descriptors;

for j = 1:num_kp

    % generate TEMPLATE xyz descriptor
    x_tmp = descriptors.cluster{j}(:, 1);
    y_tmp = descriptors.cluster{j}(:, 2);
    x_tmp = x_tmp - descriptors.kp(j, 1); % tr to kp reference
    y_tmp = y_tmp - descriptors.kp(j, 2);

    % generate SOURCE image descriptor 
    source = data_prep.img_canny;
    objective = data_prep.img_sobel;

    % generate sobel-KEYPOINTs
    threshold = params.threshold_sbl;
    clear source_msk;
    source_msk = source(descriptors.roi.p11(j, 2):descriptors.roi.p21(j, 2), ...
                        descriptors.roi.p11(j, 1):descriptors.roi.p12(j, 1));
    [kp_y, kp_x] = find(source_msk > threshold);
    kp_y = kp_y + descriptors.roi.p11(j, 2); %translate to image coord.
    kp_x = kp_x + descriptors.roi.p11(j, 1);
    N = length(kp_y);
    
    % generate sobel-PAIRs
    threshold = params.threshold_sbl;
    clear source_msk;
    source_msk = source(descriptors.roip.p11(j, 2):descriptors.roip.p21(j, 2), ...
                        descriptors.roip.p11(j, 1):descriptors.roip.p12(j, 1));
    [pair_y, pair_x] = find(source_msk > threshold);
    pair_y = pair_y + descriptors.roip.p11(j, 2); %translate to image coord.
    pair_x = pair_x + descriptors.roip.p11(j, 1);
    N2 = length(pair_y);
    
%     % GPU preparation
%     x_tmp = gpuArray(repmat(x_tmp, N2, 1));
%     y_tmp = gpuArray(repmat(y_tmp, N2, 1));
%     pair_y = repmat(pair_y', M, 1);
%     pair_y = gpuArray(pair_y(:));
%     pair_x = repmat(pair_x', M, 1);
%     pair_x = gpuArray(pair_x(:));

    % search sobel-KEYPOINTs-PAIRs that maximize cost function
    clear vector;
    vector = zeros(1, N*N2);
    dist_tmplt = descriptors.distance(j);
    rot_tmplt = descriptors.rotation(j);
    w = params.camera_params.ImageSize(2);
    h = params.camera_params.ImageSize(1);
    dist_min = dist_tmplt * params.dist_factor;
    dist_max = dist_tmplt +  dist_tmplt * (1 - params.dist_factor);
    rot_max = params.rot_max;  
    disp('*** init pair search ***')
    t = tic;
    parfor ii = 1:N*N2 
        id_kp = floor((ii-1) / N2) + 1;
        id_pair = mod(ii-1, N2) + 1;
        [dist_src, rot_src, ~] = cartesian2SphericalInDegrees(pair_x(id_pair) - kp_x(id_kp), pair_y(id_pair) - kp_y(id_kp), 0);
        rot_abs = abs(rot_src - rot_tmplt);
        if rot_abs > 180
            rot_abs = abs(rot_abs - 360);
        end
        if dist_src > dist_min && dist_src < dist_max && rot_abs < rot_max

            % scale TEMPLATE points to SOURCE scale
            scale_factor = dist_src / dist_tmplt;
            x_tmp_act = x_tmp * scale_factor;
            y_tmp_act = y_tmp * scale_factor;

            % translate and rorate TEMPLATE points to SOURCE pose
            yaw = (rot_src - rot_tmplt) * (pi/180);
            tr_x_tmp = round(x_tmp_act*cos(yaw) + y_tmp_act*sin(yaw) + kp_x(id_kp));
            tr_y_tmp = round(y_tmp_act*cos(yaw) - x_tmp_act*sin(yaw) + kp_y(id_kp));

            % evaluate COST FUNCTION in pair id_kp id_pair using TEMPLATE and SOURCE
            %tr_x_tmp2 = tr_x_tmp(tr_x_tmp >= 1 & tr_x_tmp <= w & tr_y_tmp >= 1 & tr_y_tmp <= h);
            %tr_y_tmp2 = tr_y_tmp(tr_x_tmp >= 1 & tr_x_tmp <= w & tr_y_tmp >= 1 & tr_y_tmp <= h);
            %z_tmp2 = z_tmp(tr_x_tmp >= 1 & tr_x_tmp <= w & tr_y_tmp >= 1 & tr_y_tmp <= h);
            %index = sub2ind([h w], tr_y_tmp2, tr_x_tmp2);
            %vector(ii) = source(index)' * z_tmp2;
            for i = 1:length(x_tmp_act)
                if tr_x_tmp(i) >= 1 && tr_x_tmp(i) <= w && ...
                   tr_y_tmp(i) >= 1 && tr_y_tmp(i) <= h
                    vector(ii) = vector(ii) + objective(tr_y_tmp(i), tr_x_tmp(i));
                end
            end 
        end
    end
    disp(toc(t))

    %maximun in cost function
    ii = find(vector==max(max(vector)));
    id_kp = floor((ii-1) / N2) + 1;
    id_pair = mod(ii-1, N2) + 1;
    
    [m1, n1] = size(kp_x(id_kp));
    [m2, n2] = size(kp_y(id_kp));
    [m3, n3] = size(pair_x(id_pair));
    [m4, n4] = size(pair_y(id_pair));
    
    if m1 == 1 && n1 == 1 && m2 == 1 && n2 == 1 && ...
       m3 == 1 && n3 == 1 && m4 == 1 && n4 == 1
        matches.kp_src(j, 1) = kp_x(id_kp);
        matches.kp_src(j, 2) = kp_y(id_kp);
        matches.pair_src(j, 1) = pair_x(id_pair);
        matches.pair_src(j, 2) = pair_y(id_pair);
    end
    matches.kp_tmp(j, 1) = descriptors.kp(j, 1);
    matches.kp_tmp(j, 2) = descriptors.kp(j, 2);
    matches.pair_tmp(j, 1) = descriptors.pair(j, 1);
    matches.pair_tmp(j, 2) = descriptors.pair(j, 2);

end

end