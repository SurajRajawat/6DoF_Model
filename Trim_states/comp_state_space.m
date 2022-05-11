function [A_lon,B_lon,A_lat,B_lat, A, B] = comp_state_space(filename,x_trim,u_trim)

% x_trim is the trimmed state,
% u_trim is the trimmed input
  
  
    [A,B,C,D]=linmod(filename,x_trim,u_trim);
    % similarity transformation
    T_lat = zeros(5,12);
    T_lat(1,5) = 1;
    T_lat(2,10) = 1;
    T_lat(3,12) = 1;
    T_lat(4,7) = 1;
    T_lat(5,9) = 1;
    A_lat = T_lat*A*T_lat';
    B_lat = T_lat*B;
    T_lat

    T_lon = zeros(5,12);
    T_lon(1,4) = 1;
    T_lon(2,6) = 1;
    T_lon(3,11) = 1;
    T_lon(4,8) = 1;
    T_lon(5,3) = 1;
    A_lon = T_lon*A*T_lon';
    B_lon = T_lon*B;
