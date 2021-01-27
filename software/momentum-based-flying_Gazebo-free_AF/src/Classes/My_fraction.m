classdef My_fraction < handle
    %calculate the fraction 
    %   Detailed explanation goes here
    
    properties
        numerator;
        denominator;
    end
    
    methods
        
        
        function result = fraction(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            result = obj.numerator/obj.denominator;
        end
    end
end

