function tf = generateMisscalibration(tf)
xyz = randn(1, 3) * 0.0;
yaw = 0;
pitch = 0;
roll = 0;
rot_matrix = angle2dcm(yaw, pitch, roll);
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);

tf.T = tf_matrix * tf.T;
end

