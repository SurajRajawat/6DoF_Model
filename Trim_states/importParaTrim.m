clear
clc
close all
% Mav aerodynamic properties
MAV.gravity = 9.81;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAVarams for Aersonade UAV
%physical parameters of airframe
MAV.mass = 13.5;
MAV.Jx   = 0.8244;
MAV.Jy   = 1.135;
MAV.Jz   = 1.759;
MAV.Jxz  = .1204;
% aerodynamic coefficients
MAV.S_wing        = 0.55;
MAV.b             = 2.8956;
MAV.c             = 0.18994;
MAV.S_prop        = 0.2027;
MAV.rho           = 1.2682;
MAV.k_motor       = 80;
MAV.k_T_MAV         = 0;
MAV.k_Omega       = 0;
MAV.e             = 0.9;

MAV.C_L_0         = 0.28;
MAV.C_L_alpha     = 3.45;
MAV.C_L_q         = 0.0;
MAV.C_L_delta_e   = -0.36;
MAV.C_D_0         = 0.03;
MAV.C_D_alpha     = 0.30;
MAV.C_D_p         = 0.0437;
MAV.C_D_q         = 0.0;
MAV.C_D_delta_e   = 0.0;
MAV.C_m_0         = -0.02338;
MAV.C_m_alpha     = -0.38;
MAV.C_m_q         = -3.6;
MAV.C_m_delta_e   = -0.5;
MAV.C_Y_0         = 0.0;
MAV.C_Y_beta      = -0.98;
MAV.C_Y_p         = 0.0;
MAV.C_Y_r         = 0.0;
MAV.C_Y_delta_a   = 0.0;
MAV.C_Y_delta_r   = -0.17;
MAV.C_ell_0       = 0.0;
MAV.C_ell_beta    = -0.12;
MAV.C_ell_p       = -0.26;
MAV.C_ell_r       = 0.14;
MAV.C_ell_delta_a = 0.08;
MAV.C_ell_delta_r = 0.105;
MAV.C_n_0         = 0.0;
MAV.C_n_beta      = 0.25;
MAV.C_n_p         = 0.022;
MAV.C_n_r         = -0.35;
MAV.C_n_delta_a   = 0.06;
MAV.C_n_delta_r   = -0.032;
MAV.C_prop        = 1.0;
MAV.M             = 50;
MAV.epsilon       = 0.1592;
MAV.alpha0        = 0.4712;

% wind parameters
MAV.wind_n = 0;%3;
MAV.wind_e = 0;%2;
MAV.wind_d = 0;
MAV.L_u = 200;
MAV.L_v = 200;
MAV.L_w = 50;
MAV.sigma_u = 1.06; 
MAV.sigma_v = 1.06;
MAV.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
MAV.Va0 = 16;
gamma =0*pi/180;  % desired flight path angle (radians)
R     = 100000;         % desired radius (m) - use (+) for right handed orbit, 
                     %                          (-) for left handed orbit
% autopilot sample rate
MAV.Ts = 0.01;

% first cut at initial conditions
MAV.pn0    = 0;  % initial North position
MAV.pe0    = 0;  % initial East position
MAV.pd0    = 0;  % initial Down position (negative altitude)
MAV.u0     = MAV.Va0; % initial velocity along body x-axis
MAV.v0     = 0;  % initial velocity along body y-axis
MAV.w0     = 0;  % initial velocity along body z-axis
MAV.phi0   = 0;  % initial roll angle
MAV.theta0 = 0;  % initial pitch angle
MAV.psi0   = 0;  % initial yaw angle
MAV.p0     = 0;  % initial body frame roll rate
MAV.q0     = 0;  % initial body frame pitch rate
MAV.r0     = 0;  % initial body frame yaw rate

% Gamma parameters
MAV.Gamma  = MAV.Jx*MAV.Jz-MAV.Jxz^2;
MAV.Gamma1 = (MAV.Jxz*(MAV.Jx-MAV.Jy+MAV.Jz))/MAV.Gamma;
MAV.Gamma2 = (MAV.Jz*(MAV.Jz-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma3 = MAV.Jz/MAV.Gamma;
MAV.Gamma4 = MAV.Jxz/MAV.Gamma;
MAV.Gamma5 = (MAV.Jz-MAV.Jx)/MAV.Jy;
MAV.Gamma6 = MAV.Jxz/MAV.Jy;
MAV.Gamma7 = (MAV.Jx*(MAV.Jx-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma8 = MAV.Jx/MAV.Gamma;

% run trim commands
[x_trim, u_trim]=comp_trim('model_trim',MAV.Va0,gamma,R, MAV);
MAV.u_trim = u_trim;
MAV.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
MAV.pn0    = 0;  % initial North position
MAV.pe0    = 0;  % initial East position
MAV.pd0    = 0;  % initial Down position (negative altitude)
MAV.u0     = x_trim(4);  % initial velocity along body x-axis
MAV.v0     = x_trim(5);  % initial velocity along body y-axis
MAV.w0     = x_trim(6);  % initial velocity along body z-axis
MAV.phi0   = x_trim(7);  % initial roll angle
MAV.theta0 = x_trim(8);  % initial pitch angle
MAV.psi0   = x_trim(9);  % initial yaw angle
MAV.p0     = x_trim(10);  % initial body frame roll rate
MAV.q0     = x_trim(11);  % initial body frame pitch rate
MAV.r0     = x_trim(12);  % initial body frame yaw rate

% trim inputs
MAV.del_e   = u_trim(1);
MAV.del_a   = u_trim(2);
MAV.del_r   = u_trim(3);
MAV.del_t   = u_trim(4);


% State space model
[A_lon,B_lon,A_lat,B_lat, A, B] = comp_state_space('model_trim',x_trim,u_trim);

%% Transfer function model
% transfer function parameter 
MAV.C_p_p = MAV.Gamma3*MAV.C_ell_p + MAV.Gamma4*MAV.C_n_p;
MAV.C_p_delta_a = MAV.Gamma3*MAV.C_ell_r + MAV.Gamma4*MAV.C_n_delta_a;

%% Roll function - Lateral
% Time dependent
% phi.. = -a_phi1*phi. + a_phi_2*delta_a + d_phi_2

% Laplacian space
% phi(s) = (a_phi2/s(s+a_phi1))*(delta_a(s) + d_phi_2(s)/a_phi_2)

MAV.a_phi1 = -1/2*MAV.rho*MAV.Va0^2*MAV.S_wing*MAV.b*MAV.C_p_p*(MAV.b/(2*MAV.Va0));% 1/2 rho va^2 S b Cpp b/2va
MAV.a_phi2 = 1/2*MAV.rho*MAV.Va0^2*MAV.S_wing*MAV.b*MAV.C_p_delta_a;% 1/2 rho va^2 S b Cp deltaa

%d_phi_2 = 

%% Course correction -Lateral

% Time dependent 
% chi. = g/Vg(phi + dchi)

% Laplacian space
% chi(s) = (g/Vg)/s * (phi(s) + dchi(s))
% In the absence of wind, we have Va = Vg and psi = chi
MAV.Va_trim = MAV.u0; 
% d_chi = 

%% Sideslip - Lateral

% Time dependent
% beta. = -a_beta1*beta + a_beta2*delta_r + d_beta
% β. = −aβ1*β + aβ2*δr + dβ

% Laplacian domain
% β(s) = aβ2/(s + aβ1)*(δr(s) + dβ (s))

%aβ1 = −ρVa S/(2m)*CYβ
%aβ2 = ρVa S/2m*CYδr
%dβ = 1/Va(pw − r u + g cos θ sin φ) + ρVa S/2m*(CY0 + CYp*(bp/2Va) +
%CYr(br/2Va) + CYδa δa

MAV.a_beta1 =  -1/(2*MAV.mass)*MAV.rho*MAV.Va0*MAV.S_wing*MAV.C_Y_beta;
MAV.a_beta2 = 1/(2*MAV.mass)*MAV.rho*MAV.Va0*MAV.S_wing*MAV.C_Y_delta_r;
% d_beta  = 

%% Longitudinal transfer function
% The variables of interest are the pitch angle θ, the pitch rate q, the altitude h = −pd, and the airspeed Va.
% Control signal s are the elevator δe and the throttle δt

%% Pitch angle
% θ¨ = −aθ1*θ. − aθ2*θ + aθ3*δe + dθ2
% aθ1 = −ρVa^2*cS/2Jy*Cmq*(c/2Va)
% aθ2 = −ρVa^2*cS/2Jy*Cmα
% aθ3 = −ρVa^2*cS/2Jy*Cmδe
% dθ2 = Gamma6(r^2 − p^2) + Gamma5*pr + ρVa^2*cS/2Jy(Cm0 − Cmα*γ −
% Cmq*c/2Va*dθ1) + dθ1.

% Laplacian domain
% θ(s) = (aθ3/(s2 + aθ1 s + aθ2))*(δe(s) + 1/aθ3(dθ2(s))

MAV.a_theta1 = -MAV.rho*MAV.Va0^2*MAV.S_wing*(MAV.c/(2*MAV.Jy))*MAV.C_m_q*(MAV.c/(2*MAV.Va0));
MAV.a_theta2 = -MAV.rho*MAV.Va0^2*MAV.S_wing*(MAV.c/(2*MAV.Jy))*MAV.C_m_alpha;
MAV.a_theta3 = -MAV.rho*MAV.Va0^2*MAV.S_wing*(MAV.c/(2*MAV.Jy))*MAV.C_m_delta_e;

%% Altitude
% For a constant airspeed, the pitch angle directly influences the climb rate of the aircraft. 
% h. = Vaθ + dh
% dh = (u sin θ − Vaθ) − v sin φ cos θ − w cos φ cos θ

% Laplacian domain
% h(s) = Va/s*(θ +1/Va*dh) -->  dynamics from
% the elevator to the altitude , Va constant, θ input

%  h(s) = θ/s*(Va +1/θ*dh)--> m airspeed to altitude, hold θ constant in equation, Va as input
%dh = 
MAV.theta_trim = MAV.theta0;

%% Airspeed
% Va. = u.*cosα + w.sinα + dV1
% dV1 = −u.(1 − cos β) cos α − w.(1 − cos β) sin α + v. sin β -->( when β =
% 0. dV1 = 0 aswell)

% ¯ --> vector
% ∗ --> trim cond
% V¯.a = −aV1V¯a + aV2*δ¯t − aV3*θ¯ + dV
% aV1 = ρVa∗S/m(CD0 + CDα α∗ + CDδe δe∗) + ρSprop/m*CpropVa∗
% aV2 = ρSprop/m*Cpropk^δt∗
% aV3 = gcos(θ ∗ − χ∗)

% Laplacian domain
% V¯a(s) = 1(s + aV1)*(aV2 δ¯t(s) − aV3 θ¯(s) + dV(s))
% MAV.alpha0 is not trim 

MAV.a_V1 = 1/(MAV.mass)*MAV.rho*MAV.u0*MAV.S_wing*(MAV.C_D_0 + MAV.C_D_alpha*MAV.alpha0 + MAV.C_D_delta_e*MAV.del_e) + MAV.rho*(MAV.S_prop/MAV.mass)*MAV.C_prop*MAV.u0;
MAV.a_V2 = MAV.rho*MAV.S_prop/(MAV.mass)*MAV.C_prop*MAV.del_t;
MAV.a_V3 = MAV.gravity*cos(MAV.theta0-MAV.psi0);

[T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_v_delta_r] = comp_tran_func(MAV);

