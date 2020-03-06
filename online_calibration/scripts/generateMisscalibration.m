function [tf, gt] = generateMisscalibration()

xyz = [-0.07 0.06 0.05];
yaw = 0.8;
pitch = -0.4;
roll = 0.4;
rot_matrix = angle2dcm(yaw*(pi/180), pitch*(pi/180), roll*(pi/180));
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);

tf = affine3d;
tf.T = tf_matrix;

gt = [xyz yaw pitch roll];

end

