function v_wind=generate_wind(t)

v_wind=zeros(3,1);
simu_windgust=true;
v_const=3;%10; constant wind speed
v_gust=13; % wind gust 
v_raise=v_gust-v_const; % difference between wind gust and constant wind speed

sigma=2; % sigma for gaussian distribution
mu=25; % mean value for gaussian distribution 

%generate gaussian distributed wind along time to simulate the wind gust 
 if simu_windgust

   v_wind=[-(v_const+v_raise*exp(-(t - mu).^2/(2*sigma^2)));0;0];

 else
    v_wind=[-5;0;0]; % constnt wind 
 end

end