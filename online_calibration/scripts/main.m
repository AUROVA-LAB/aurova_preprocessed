clear, clc, close all

%****************** global variables ******************%
id_dataset = 5;
id_sample = 380;
id_pair = 1;
sigma = 20;
sigma_flt = 5;
sobel_fact = 0.5;
base = 255;
threshold_dw = 0.0;
threshold_up = 20;
area = 200;
exec_flag = [true true true true];
filenames_cell{1} = 'raw_data/sec_0101/';
filenames_cell{2} = 'raw_data/sec_0105/';
filenames_cell{3} = 'raw_data/sec_0107/';
filenames_cell{4} = 'raw_data/sec_0202/';
filenames_cell{5} = 'raw_data/sec_0203/';

%************* read and store raw data ****************%
% TODO: return pc, images, etc, instead to cells
if exec_flag(1)
    [scans_lidarframe, scans_mapframe, tfs_lidar2map, tfs_map2camera, image] = readData(filenames_cell{id_dataset}, id_sample);
    [h, w, c] = size(image);
end

%************ preprocess camera data image ***************%
if exec_flag(2)
    image_gray = rgb2gray(image);
    [image_grad, image_dir] = imgradient(image_gray, 'sobel');
    image_grad_flt = imgaussfilt(image_grad, sigma_flt);
    image_grad_flt = image_grad * sobel_fact + image_grad_flt;
    image_grad_nrm = image_grad_flt / max(max(image_grad_flt));
    image_grad_plt = uint8(image_grad_nrm * base);
    figure
    imshow(image_grad_plt)
end

%************** filtering of scan lidar *******************%
if exec_flag(2)
    st_lidar_cfg = fillLidarCfg(scans_lidarframe{1});
    scan_filtered = filterScanAzimuth(scans_lidarframe{1}, st_lidar_cfg, threshold_dw, threshold_up, base);
end

%********** project 3D points into 2D pixel plane ********%
if exec_flag(3)
    [image_depth, image_discnt] = imageDepthFromLidar(scan_filtered, tfs_lidar2map{1}, tfs_map2camera{1}, sigma, base);
    image_discnt_plt(:, :, 1) = image_grad_plt;
    image_discnt_plt(:, :, 2) = image_grad_plt;
    image_discnt_plt(:, :, 3) = image_grad_plt;
    [v_array, u_array] = find(image_discnt > 10);
    k_array(1:length(v_array), 1) = 1;
    image_discnt_plt = insertShape(image_discnt_plt, 'circle', [u_array, v_array, k_array], 'LineWidth', 1, 'Color', 'green');

    figure
    imshow(image_depth)
    figure
    imshow(image_discnt)
end

%********* TODO: encapsule this block in functions!! **********%
% introduction of points manually
if exec_flag(4)
    [p1_tmplt, p2_tmplt, p11_src, p12_src, p21_src] = manuallyKeyPoints(id_dataset, id_sample, id_pair);

    % scale and rotation info of lidar pair
    p0_tmplt = p2_tmplt - p1_tmplt;
    [dist_tmplt, rot_tmplt, ele] = cartesian2SphericalInDegrees(p0_tmplt(1), p0_tmplt(2), 0);
    source = image_grad(p11_src(2):p21_src(2), p11_src(1):p12_src(1));
    dist_min = dist_tmplt * (2/3);
    dist_max = dist_tmplt * (3/2);

    % keypoints lidar in pc format centered around p1_tmplt
    inix = p1_tmplt(1) - area;
    if inix < 1 
        inix = 1;
    end
    endx = p1_tmplt(1) + area;
    if endx > w
        endx = w;
    end
    iniy = p1_tmplt(2) - area;
    if iniy < 1 
        iniy = 1;
    end
    endy = p1_tmplt(2) + area;
    if endy > h
        endy = h;
    end
    template_discnt = image_discnt;%(iniy:endy, inix:endx);
    [pt_y_tm, pt_x_tm] = find(template_discnt > 10); % cloud template
    pt_z_tm(1:length(pt_x_tm), 1) = double(0);
    pt_x_tm = pt_x_tm - p1_tmplt(1);% + inix;
    pt_y_tm = pt_y_tm - p1_tmplt(2);% + iniy;
    pt_xyz_tm = [pt_x_tm'; pt_y_tm'; pt_z_tm']';
    pt_cloud_tm = pointCloud(pt_xyz_tm);

    % N keypoints image in array format
    [kp_y, kp_x] = find(source > 10);
    kp_y = kp_y + p11_src(2);
    kp_x = kp_x + p11_src(1);
    kp_src = cat(2, kp_y, kp_x);
    N = length(kp_y);

    %*********************
    matrix(1:N, 1:N) = double(0);
    for n1 = 1:N
        for n2 = 1:N
            % scale and rotation info of image pair
            p00_src = kp_src(n2, :) - kp_src(n1, :);
            [dist_src, rot_src, ele] = cartesian2SphericalInDegrees(p00_src(2), p00_src(1), 0);
            if dist_src > dist_min && dist_src < dist_max
                % scale template points
                scale_factor = dist_src / dist_tmplt;
                pt_x_tm_act = pt_x_tm * scale_factor;
                pt_y_tm_act = pt_y_tm * scale_factor;
                pt_xyz_tm = [pt_x_tm_act'; pt_y_tm_act'; pt_z_tm']';

                % translate and rorate points
                x = kp_src(n1, 2);
                y = kp_src(n1, 1);
                z = 0;
                roll = 0;
                pitch = 0;
                yaw = (rot_src - rot_tmplt) * (pi/180);
                xyz_rpy = [x, y, z, roll, pitch, yaw];
                tf = getTfMatrix(xyz_rpy, 1);
                for i = 1:length(pt_x_tm_act)
                    pt = cat(2, pt_xyz_tm(i, :), 1);
                    pt = pt * tf;
                    if pt(1) >= 1 && pt(1) <= w && pt(2) >= 1 && pt(2) <= h
                        u = round(pt(1));
                        v = round(pt(2));
                        matrix(n1, n2) = matrix(n1, n2) + image_grad(v, u);
                    end
                end 
            end
        end
    end

    [N1, N2] = find(matrix==max(max(matrix)));
    for n1 = N1:N1
        for n2 = N2:N2
            % scale and rotation info of image pair
            p00_src = kp_src(n2, :) - kp_src(n1, :);
            [dist_src, rot_src, ele] = cartesian2SphericalInDegrees(p00_src(2), p00_src(1), 0);
            if dist_src > dist_min && dist_src < dist_max
                % scale template points
                scale_factor = dist_src / dist_tmplt;
                pt_x_tm_act = pt_x_tm * scale_factor;
                pt_y_tm_act = pt_y_tm * scale_factor;
                pt_xyz_tm = [pt_x_tm_act'; pt_y_tm_act'; pt_z_tm']';
                %pt_cloud_tm = pointCloud(pt_xyz_tm);

                % translate and rorate points
                x = kp_src(n1, 2);
                y = kp_src(n1, 1);
                z = 0;
                roll = 0;
                pitch = 0;
                yaw = (rot_src - rot_tmplt) * (pi/180);
                xyz_rpy = [x, y, z, roll, pitch, yaw];
                tf = getTfMatrix(xyz_rpy, 1);
                image_match_plt(:, :, 1) = image_grad_plt;
                image_match_plt(:, :, 2) = image_grad_plt;
                image_match_plt(:, :, 3) = image_grad_plt;
                for i = 1:length(pt_x_tm_act)
                    pt = cat(2, pt_xyz_tm(i, :), 1);
                    pt = pt * tf;
                    if pt(1) >= 1 && pt(1) <= w && pt(2) >= 1 && pt(2) <= h
                        u = round(pt(1));
                        v = round(pt(2));
                        k = 1;
                        image_match_plt = insertShape(image_match_plt, 'circle', [u, v, k], 'LineWidth', 1, 'Color', 'green');
                        matrix(n1, n2) = matrix(n1, n2) + image_grad(v, u);
                    end
                end 
                k = 2;
                u = kp_src(n1, 2);
                v = kp_src(n1, 1);
                image_match_plt = insertShape(image_match_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'red');
                u = kp_src(n2, 2);
                v = kp_src(n2, 1);
                image_match_plt = insertShape(image_match_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'red');
                u = p1_tmplt(1);
                v = p1_tmplt(2);
                image_match_plt = insertShape(image_match_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');
                u = p2_tmplt(1);
                v = p2_tmplt(2);
                image_match_plt = insertShape(image_match_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');
                u = p1_tmplt(1);
                v = p1_tmplt(2);
                image_discnt_plt = insertShape(image_discnt_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');
                u = p2_tmplt(1);
                v = p2_tmplt(2);
                image_discnt_plt = insertShape(image_discnt_plt, 'circle', [u, v, k], 'LineWidth', 2, 'Color', 'yellow');
                figure
                imshow(image_discnt_plt)
                figure
                imshow(image_match_plt)
            end
        end
    end
end

% % BACKUP:
% %****************** object segmentation ****************% T = 70 s
% if exec_flag(3)
%     [objects_gray, objects_depth] = imageSegmentation(image, image_depth, index, read_mat(3));
%     figure
%     montage(objects_gray)
%     figure
%     montage(objects_depth)
% end
% 
% %********** objects corregistration scans-image **********% T = 15 s
% if exec_flag(4)
%     plot_gray = objectsCorregistration(objects_gray, objects_depth, image);
%     figure
%     imshow(plot_gray)
% end