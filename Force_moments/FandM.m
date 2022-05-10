%   FandM.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = FandM(x,delta,wind,MAV)
    pn       = x(1);       % inertial North position     
    pe       = x(2);       % inertial East position
    pd       = x(3);           
    u        = x(4);       
    v        = x(5);       
    w        = x(6);       
    phi      = x(7);       % roll angle         
    theta    = x(8);       % pitch angle     
    psi      = x(9);       % yaw angle     
    p        = x(10);       % roll rate
    q        = x(11);       % pitch rate     
    r        = x(12);       % yaw rate
    delta_e  = delta(1);
    delta_a  = delta(2);
    delta_r  = delta(3);
    delta_t  = delta(4);
    wind_ns  = wind(1);
    wind_es  = wind(2);
    wind_ds  = wind(3);
    wind_ug  = wind(4);
    wind_vg  = wind(5);
    wind_wg  = wind(6);
    
    mass     = MAV.mass;
    grav     = MAV.gravity;
    dens     = MAV.rho;
    S        = MAV.S_wing;
    
    %     % process inputs to function
%     pn       = 1;       % inertial North position     
%     pe       = 1;       % inertial East position
%     pd       = 1;           
%     u        = 0;       
%     v        = 0;       
%     w        = 0;       
%     phi      = 0;       % roll angle         
%     theta    = 0;       % pitch angle     
%     psi      = 0;       % yaw angle     
%     p        = 0;       % roll rate
%     q        = 0;       % pitch rate     
%     r        = 0;       % yaw rate    
%     t        = 0;       % time
%     delta_e  = 1;
%     delta_a  = 0;
%     delta_r  = 0;
%     delta_t  = 0;
%     wind_ns  = 2;
%     wind_es  = 2;
%     wind_ds  = 3;
%     wind_ug  = 4;
%     wind_vg  = 5;
%     wind_wg  = 6;    
%t        = x(13);       % time
    
    % Rotate inertial frame to body frame for wind
%     R_roll = [...
%           1, 0, 0;...
%           0, cos(phi), -sin(phi);...
%           0, sin(phi), cos(phi)];
%     R_pitch = [...
%           cos(theta), 0, sin(theta);...
%           0, 1, 0;...
%           -sin(theta), 0, cos(theta)];
%     R_yaw = [...
%           cos(psi), -sin(psi), 0;...
%           sin(psi), cos(psi), 0;...
%           0, 0, 1];
    Rb_v = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
            cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
            -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]';  
        
    
    
    wn = wind_ns + [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)]*[wind_ug wind_vg wind_wg]';
    we = wind_es + [cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)]*[wind_ug wind_vg wind_wg]';
    wd = wind_ds + [-sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)]*[wind_ug wind_vg wind_wg]';
        
    %relative velocity wrt airspeed
    Vr = [u, v, w]' - Rb_v*[wind_ns, wind_es, wind_ds]' - [wind_ug, wind_vg, wind_wg]';
    
    
    Va = norm(Vr);
    alpha = atan2(Vr(3),Vr(1));
    beta  = asin(Vr(2)/Va);
%     wn    = 0;
%     we    = 0;
%     wd    = 0;
%     
    % Constants
    C_x_alpha = -MAV.C_D_alpha*cos(alpha) + MAV.C_L_alpha*sin(alpha);
    C_x_q     = -MAV.C_D_q*cos(alpha) + MAV.C_L_q*sin(alpha);
    C_x_delta = -MAV.C_D_delta_e*cos(alpha) + MAV.C_L_delta_e*sin(alpha);
    C_z_alpha = -MAV.C_D_alpha*sin(alpha) - MAV.C_L_alpha*cos(alpha);
    C_z_q     = -MAV.C_D_q*sin(alpha) - MAV.C_L_q*cos(alpha);
    C_z_delta = -MAV.C_D_delta_e*sin(alpha) - MAV.C_L_delta_e*cos(alpha);
    % Forces and Moments
    
    % Non linearised
    Force(1)  = -mass*grav*sin(theta) + 1/2*dens*Va^2*S*(C_x_alpha + C_x_q*(MAV.c/(2*Va)*q +C_x_delta*delta_e) + 1/2*MAV.rho*MAV.S_prop*MAV.C_prop*((MAV.k_motor*delta_t)^2-Va^2));
    Force(2)  = -mass*grav*cos(theta)*sin(phi) + 1/2*dens*((Va)^2)*S*(MAV.C_Y_0 + MAV.C_Y_beta*beta + MAV.C_Y_p*MAV.b/(2*Va)*p + MAV.C_Y_r*MAV.b/(2*Va)*r + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);
    Force(3)  = -mass*grav*cos(theta)*cos(phi) + 1/2*dens*((Va)^2)*S*(C_z_alpha + C_z_q*MAV.c/2/Va*q + C_z_delta*delta_e);
    
    %bCheck all variables are correct
    Torque(1) = 1/2*MAV.rho*Va^2*S*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta*beta + MAV.C_ell_p*MAV.b/2/Va*p + MAV.C_ell_r*MAV.b/2/Va*r + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r*delta_r) - MAV.k_T_MAV*MAV.k_Omega^2*delta_t^2;
    Torque(2) = 1/2*MAV.rho*Va^2*S*MAV.c*(MAV.C_m_0 + MAV.C_m_alpha*alpha + MAV.C_m_q*MAV.c/2/Va*q + MAV.C_m_delta_e*delta_e);   
    Torque(3) = 1/2*MAV.rho*Va^2*S*MAV.b*(MAV.C_n_0 + MAV.C_n_beta*beta + MAV.C_n_p*MAV.b/2/Va*p + MAV.C_n_r*MAV.b/2/Va*r + MAV.C_n_delta_a*delta_a + MAV.C_n_delta_r* delta_r);
    
    
    out = [Force'; Torque'; Va; alpha; beta; wn; we; wd];
end