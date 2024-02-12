function [X_resampled, Y_resampled] = resampleXYGrid(X, Y, n)
    
% n must be rounded to the closest square integer
n = floor(sqrt(n));

x_resampled_border = linspace(X(1,1), X(1,end), n);
y_resampled_border = linspace(Y(1,1), Y(end,1), n);

[X_resampled,Y_resampled] = meshgrid(x_resampled_border,y_resampled_border);

% x_resampled_mid = zeros(1,n-1);
% y_resampled_mid = zeros(1,n-1);
% 
% for i=1:(length(x_resampled_border)-1)
%     x_resampled_mid(i) = (x_resampled_border(i)+x_resampled_border(i+1))/2;
%     y_resampled_mid(i) = (y_resampled_border(i)+y_resampled_border(i+1))/2;
% end
% 
% % [X_resampled_mid,Y_resampled_mid] = meshgrid(x_resampled_mid,y_resampled_mid);
% 
% x_resampled = zeros((length(x_resampled_border)+length(x_resampled_mid)),1);
% y_resampled = zeros((length(y_resampled_border)+length(y_resampled_mid)),1);
% 
% for i=1:(length(x_resampled_border)+length(x_resampled_mid))
%     if mod(i,2)~=0
%         x_resampled(i)= x_resampled_border(round(i/2));
%         y_resampled(i)= y_resampled_border(round(i/2));
%     else
%         x_resampled(i)= x_resampled_mid(i/2);
%         y_resampled(i)= y_resampled_mid(i/2);
%     end
% end
% 
% [X_resampled,Y_resampled] = meshgrid(x_resampled,y_resampled);
% 
% save Resample