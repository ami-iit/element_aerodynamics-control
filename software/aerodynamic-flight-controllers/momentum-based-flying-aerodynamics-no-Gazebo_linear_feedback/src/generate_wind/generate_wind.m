%this function is used to simulate the wind
%@author: HUI TONG

function v_wind=generate_wind(t,aerodynamics_config)

v_wind=zeros(3,1); %size of wind velocity vector

v_min=3; %constant wind speed
v_max=15; % wind gust peak value
v_raise=v_max-v_min; % difference between wind gust and constant wind speed
theta_wind=pi; %wind direction (in plane X-Y)
k_wind=[cos(theta_wind) sin(theta_wind) 0]'; %wind direction unit vector

if aerodynamics_config.wind_type==1 %static wind

   v_wind=15*k_wind;%v_min*k_wind;
elseif aerodynamics_config.wind_type==2 %ramp gust
   if t<5
       v_wind=v_min*k_wind;
   elseif t>=5&&t<10
       v_amp=v_min+v_raise*((t-5)/(10-5));
       v_wind=v_amp*k_wind;
   elseif t>=10
       v_wind=v_max*k_wind;
   end
elseif aerodynamics_config.wind_type==3 %cos gust
    if t<5
        v_wind=v_min*k_wind;
    elseif t>=5&&t<20
        v_amp=v_min+v_raise/2*(1-cos((t-5)/(20-5)*2*pi));
        v_wind=v_amp*k_wind;
    elseif t>=20
        v_wind=v_min*k_wind;
    end

elseif aerodynamics_config.wind_type==4 % random wind
    
end
end


 


