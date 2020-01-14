function [src0_x, src0_y] = gpuProofs(source_x_1, source_x_2, source_y_1, source_y_2)

src0_x = source_x_1 - source_x_2;
src0_y = source_y_1 - source_y_2;

for i = 1:3000000
    a = sqrt(source_x_1);
    b = atan2(source_x_1, source_x_2);
end

end

