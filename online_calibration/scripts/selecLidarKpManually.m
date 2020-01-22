function descriptor = selectKeyPointsManually(experiments)

descriptor = [];
descriptor.kp = [];
descriptor.pair = [];
descriptor.distance = 0;
descriptor.rotation = 0;
descriptor.roi = [];
descriptor.roi.p11 = [];
descriptor.roi.p12 = [];
descriptor.roi.p21 = [];


%********************* from ROS *********************%
%****************************************************%
p1_tmplt_array{1, 500} = [34 95; 244 108];
p2_tmplt_array{1, 500} = [45 96; 252 132];
p11_src_array{1, 500} = [21 72; 228 92];
p12_src_array{1, 500} = [56 72; 268 92];
p21_src_array{1, 500} = [21 108; 228 143];

% 1 - WR (Try weigh or other local max)
p1_tmplt_array{1, 800} = [407 170];
p2_tmplt_array{1, 800} = [417 169];
p11_src_array{1, 800} = [381 142];
p12_src_array{1, 800} = [415 142];
p21_src_array{1, 800} = [381 185];

% 2 - WR (Try weigh or other local max)
p1_tmplt_array{2, 100} = [264 190; 541 232; 137 214];
p2_tmplt_array{2, 100} = [245 206; 565 232; 148 214];
p11_src_array{2, 100} = [234 182; 528 208; 116 184];
p12_src_array{2, 100} = [272 182; 567 208; 152 184];
p21_src_array{2, 100} = [234 213; 528 251; 116 209];

p1_tmplt_array{4, 1250} = [184 223; 607 165];
p2_tmplt_array{4, 1250} = [184 244; 613 145];
p11_src_array{4, 1250} = [132 214; 556 148];
p12_src_array{4, 1250} = [167 214; 584 148];
p21_src_array{4, 1250} = [132 257; 556 186];

% 2 - OK/Reg
p1_tmplt_array{4, 340} = [182 126; 176 217; 84 180];
p2_tmplt_array{4, 340} = [182 109; 167 234; 94 179];
p11_src_array{4, 340} = [145 82; 119 191; 36 141];
p12_src_array{4, 340} = [161 82; 150 191; 65 141];
p21_src_array{4, 340} = [145 132; 119 212; 36 164];

p1_tmplt_array{5, 380} = [271 261];
p2_tmplt_array{5, 380} = [279 277];
p11_src_array{5, 380} = [225 235];
p12_src_array{5, 380} = [243 235];
p21_src_array{5, 380} = [225 278];

p1_tmplt_array{5, 260} = [145 259; 447 157; 220 225];
p2_tmplt_array{5, 260} = [149 247; 446 174; 252 220];
p11_src_array{5, 260} = [90 217; 385 144; 141 210];
p12_src_array{5, 260} = [122 217; 443 144; 221 210];
p21_src_array{5, 260} = [90 250; 385 195; 141 267];

%********************* from Kitti *********************%
%****************************************************%
p1_tmplt_array{1, 10} = [382 192; 917 205];
p2_tmplt_array{1, 10} = [373 204; 917 186];
p11_src_array{1, 10} = [365 191; 912 182];
p12_src_array{1, 10} = [392 191; 922 182];
p21_src_array{1, 10} = [365 208; 912 208];

p1_tmplt_array{1, 230} = [388 149; 765 210];
p2_tmplt_array{1, 230} = [390 159; 763 196];
p11_src_array{1, 230} = [385 146; 758 191];
p12_src_array{1, 230} = [393 146; 770 191];
p21_src_array{1, 230} = [385 162; 758 215];

p1_tmplt_array{2, 1} = [306 209; 808 198];
p2_tmplt_array{2, 1} = [303 224; 812 184];
p11_src_array{2, 1} = [298 204; 803 179];
p12_src_array{2, 1} = [311 204; 817 179];
p21_src_array{2, 1} = [298 229; 803 203];

p1_tmplt_array{2, 70} = [968 185];
p2_tmplt_array{2, 70} = [958 199];
p11_src_array{2, 70} = [953 180];
p12_src_array{2, 70} = [973 180];
p21_src_array{2, 70} = [953 204];

descriptor.kp = p1_tmplt_array{experiments.id_dataset, experiments.id_sample}(experiments.id_pair, :);
descriptor.pair = p2_tmplt_array{experiments.id_dataset, experiments.id_sample}(experiments.id_pair, :);
descriptor.roi.p11 = p11_src_array{experiments.id_dataset, experiments.id_sample}(experiments.id_pair, :);
descriptor.roi.p12 = p12_src_array{experiments.id_dataset, experiments.id_sample}(experiments.id_pair, :);
descriptor.roi.p21 = p21_src_array{experiments.id_dataset, experiments.id_sample}(experiments.id_pair, :);
dist_xy = descriptor.pair - descriptor.kp;
[descriptor.distance, descriptor.rotation, ~] = cartesian2SphericalInDegrees(dist_xy(1), dist_xy(2), 0);
end

