function [p1_tmplt, p2_tmplt, p11_src, p12_src, p21_src] = manuallyKeyPoints(id_dataset, id_sample, id_pair)

% 1 - OK, 2 - OK
p1_tmplt_array{1, 500} = [34 95; 244 108];
p2_tmplt_array{1, 500} = [45 96; 252 132];
p11_src_array{1, 500} = [21 72; 228 92];
p12_src_array{1, 500} = [56 72; 268 92];
p21_src_array{1, 500} = [21 108; 228 143];

% 1 - Try weigh or other local max
p1_tmplt_array{1, 800} = [407 170];
p2_tmplt_array{1, 800} = [417 169];
p11_src_array{1, 800} = [381 142];
p12_src_array{1, 800} = [415 142];
p21_src_array{1, 800} = [381 185];

% 1 - OK, 2 - OK
p1_tmplt_array{4, 1250} = [184 223; 607 165];
p2_tmplt_array{4, 1250} = [184 244; 613 145];
p11_src_array{4, 1250} = [132 214; 556 148];
p12_src_array{4, 1250} = [167 214; 584 148];
p21_src_array{4, 1250} = [132 257; 556 186];

% 1 - OK, 2 - OK, 3 - OK
p1_tmplt_array{5, 260} = [145 259; 447 157; 220 225];
p2_tmplt_array{5, 260} = [149 247; 446 174; 252 220];
p11_src_array{5, 260} = [90 217; 385 144; 141 210];
p12_src_array{5, 260} = [122 217; 443 144; 221 210];
p21_src_array{5, 260} = [90 250; 385 195; 141 267];



p1_tmplt = p1_tmplt_array{id_dataset, id_sample}(id_pair, :);
p2_tmplt = p2_tmplt_array{id_dataset, id_sample}(id_pair, :);
p11_src = p11_src_array{id_dataset, id_sample}(id_pair, :);
p12_src = p12_src_array{id_dataset, id_sample}(id_pair, :);
p21_src = p21_src_array{id_dataset, id_sample}(id_pair, :);

end

