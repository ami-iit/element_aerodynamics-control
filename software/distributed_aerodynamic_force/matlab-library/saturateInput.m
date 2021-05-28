function y = saturateInput(u, min, max)

    % to be imported in WBC library
    assert(isequal(size(min), size(max)), 'Min and max must be same size')

    if length(min) == 1
    
        y          = u;
        y(y > max) = max;
        y(y < min) = min;
    else
    
        assert(length(min) == length(u), 'input and saturation must have same size');
        y = u;
    
        for i = 1:length(min)
        
            if y(i) > max(i)
                
                y(i) = max(i);
                
            elseif y(i) < min(i)
            
                y(i) = min(i);
            end
        end
    end
end