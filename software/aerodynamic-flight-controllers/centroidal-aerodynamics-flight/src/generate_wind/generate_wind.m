% this function is used to simulate the wind
% @author: HUI TONG

function v_wind = generate_wind(t, Config)

v_const        = Config.constant_wind; % constant wind speed [m/s]
wind_direction = Config.wind_direction;
v_ramp_max     = Config.wind_gust_ramp;
t_init_ramp    = Config.t_init_wind_ramp;
t_slope_ramp   = Config.t_slope_wind_ramp;
t_plateau_ramp = Config.t_plateau_wind_ramp;
v_cosine_max   = Config.wind_gust_cosine;
t_init_cosine  = Config.t_init_wind_cosine;
t_delta_cosine = Config.delta_t_wind_cosine;

% parametrized ramp profile
if t > t_init_ramp && t <= (t_init_ramp + t_slope_ramp)
    
    v_ramp = v_ramp_max * ((t-t_init_ramp)/t_slope_ramp);
    
elseif t > (t_init_ramp + t_slope_ramp) && t <= (t_init_ramp + t_slope_ramp + t_plateau_ramp)
    
    v_ramp = v_ramp_max;
    
elseif t > (t_init_ramp + t_slope_ramp + t_plateau_ramp) && t <= (t_init_ramp + 2*t_slope_ramp + t_plateau_ramp)
    
    v_ramp = v_ramp_max * (1-(t-(t_init_ramp + t_slope_ramp + t_plateau_ramp))/t_slope_ramp);
    
else
    v_ramp = 0;
end

% % parametrized ramp profile (using heaviside functions)
% 
% v_ramp = v_ramp_max * ((t-t_init_ramp)/t_slope_ramp) * ...
%                             heaviside(t - t_init_ramp) * heaviside((t_init_ramp + t_slope_ramp) - t) + ...
%          v_ramp_max * ...
%                             heaviside(t - (t_init_ramp + t_slope_ramp)) * heaviside((t_init_ramp + t_slope_ramp + t_plateau_ramp) - t) + ...
%          v_ramp_max * (1-(t-(t_init_ramp + t_slope_ramp + t_plateau_ramp))/t_slope_ramp) * ...
%                             heaviside(t - (t_init_ramp + t_slope_ramp + t_plateau_ramp)) * heaviside((t_init_ramp + 2*t_slope_ramp + t_plateau_ramp) - t);

% parametrized cosine profile
if t > t_init_cosine && t <= (t_init_cosine + t_delta_cosine)
    
    v_cosine = 0.5*v_cosine_max*(1-cos((t-t_init_cosine)/t_delta_cosine*2*pi));
    
else
    v_cosine = 0;
end

% % parametrized cosine profile (using heaviside functions)
% 
% v_cosine = 0.5*v_cosine_max*(1 - cos((t - t_init_cosine)/t_delta_cosine*2*pi)) * heaviside(t - t_init_cosine) * heaviside((t_init_cosine + t_delta_cosine) - t);


% total wind profile
v_wind = (v_const + v_ramp + v_cosine) * wind_direction;

end