function [tf, gt] = generateMisscalibration()

xyz = [0.08 0.11 0.03];
yaw = 0.1;
pitch = 0.6;
roll = 0.5;
rot_matrix = angle2dcm(yaw*(pi/180), pitch*(pi/180), roll*(pi/180));
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);

tf = affine3d;
tf.T = tf_matrix;

gt = [xyz yaw pitch roll];

end

