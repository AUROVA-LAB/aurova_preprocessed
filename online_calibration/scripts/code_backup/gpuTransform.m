function [x_tr, y_tr] = gpuTransform(x, y, delta_x, delta_y, delta_theta)

x_tr = x*cos(delta_theta) + y*sin(delta_theta) + delta_x;
y_tr = -x*sin(delta_theta) + delta_y;

end

