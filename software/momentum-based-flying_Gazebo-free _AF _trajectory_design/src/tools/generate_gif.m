function generate_gif(n, h, filename)
    
    axis tight manual % this ensures that getframe() returns a consistent size
    % Capture the plot as an image
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
    if n == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', 0.005);
    end
end

