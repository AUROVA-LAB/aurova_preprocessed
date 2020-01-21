function tf_affine = getTfAffineMatrix(xyz_rpy, select)

roll = xyz_rpy(select, 4);
pitch = xyz_rpy(select, 5);
yaw = xyz_rpy(select, 6);
xyz = xyz_rpy(select, 1:3);
rot_matrix = angle2dcm(yaw, pitch, roll);
tf_matrix = cat(1, rot_matrix, xyz);
tf_matrix = cat(2, tf_matrix, [0; 0; 0; 1]);

tf_affine = affine3d(tf_matrix);

end

