function tf = generateMisscalibration()
sigma_randn = 0.15;
sigma_randn2 = 0.075;
xyz = randn(1, 3) * sigma_randn;
yaw = randn * (pi/180) * sigma_randn2;
pitch = randn * (pi/180) * sigma_randn2;
roll = randn * (pi/180) * sigma_randn2;
rot_matrix = angle2dcm(yaw, pitch, roll);
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);

tf = affine3d;
tf.T = tf_matrix;
end

