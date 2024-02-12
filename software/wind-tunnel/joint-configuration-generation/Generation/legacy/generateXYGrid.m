function [X,Y] = generateXYGrid(x_max, x_min, y_max, y_min, step_x, step_y)

x     = x_min:step_x:x_max;
y     = y_min:step_y:y_max;
[X,Y] = meshgrid(x,y);

end