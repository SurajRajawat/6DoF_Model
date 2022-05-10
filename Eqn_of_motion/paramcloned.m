P.gravity = 9.81;
   
%physical parameters of airframe
P.mass = 13.5;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = 0.1204;

% initial conditions
P.pn0    = 0;   % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% wind parameters
W.wind_e  = 0;  % initial wind speed eastward
W.wind_d  = 0;  % initial wind speed downwards
W.wind_n  = 0;  % initial wind speed northward

% compute air data
W.Va = 0;
W.alpha = 0;
W.beta = 0;
