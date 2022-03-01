% this function is used to simulate the wind
% @author: HUI TONG

function v_wind = generate_wind(t, Config)

v_const = 3 * Config.constant_wind;  % constant wind speed [m/s]
v_gust  = 10;                        % wind gust peak value [m/s]
v_raise = v_gust - v_const;          % difference between wind gust and constant wind speed
sigma   = 2;                         % sigma for gaussian distribution
mu      = 8;                         % mean value for gaussian distribution

% generate gaussian distributed wind along time to simulate the wind gust
if Config.wind_gust
    
    if t <= 20
        v_wind = [-(v_const+v_raise*exp(-(t - mu).^2/(2*sigma^2))); 0; 0];
    else
        v_wind = [-v_const; 0; 0];
    end
else
    v_wind = [-v_const; 0; 0]; % constant wind
end
end