function x_resampled = resampleLineSpace(x, n)

n = floor(sqrt(n))*floor(sqrt(n));

x_resampled = linspace(x(1),x(end),n);

end