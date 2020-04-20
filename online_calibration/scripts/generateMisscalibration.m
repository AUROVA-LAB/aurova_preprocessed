function [tf, state, gt] = generateMisscalibration()

% misscalibration
xyz = [-0.8 0.7 -0.6];
ypr = [0.8 0.9 -0.99];

% generate transform
rot_matrix = angle2dcm(ypr(1)*(pi/180), ypr(2)*(pi/180), ypr(3)*(pi/180));
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);
tf = affine3d;
tf.T = tf_matrix;

% generate initial state
state = [];
state.stt = [xyz ypr]';
state.cov = eye(length(state.stt));
state.cov(1, 1) = max(xyz);
state.cov(2, 2) = max(xyz);
state.cov(3, 3) = max(xyz);
state.cov(4, 4) = max(ypr);
state.cov(5, 5) = max(ypr);
state.cov(6, 6) = max(ypr);

% save ground truth
gt = [xyz ypr];

end

