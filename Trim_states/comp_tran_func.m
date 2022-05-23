function[T_phi_delta_a, T_chi_phi, T_theta_delta_e, T_h_theta, T_h_Va, T_Va_delta_t, T_Va_theta, T_v_delta_r] = comp_tran_func(MAV)
% x_trim is the trimmed state,
% u_trim is the trimmed input
      
    
    % define transfer functions
    T_phi_delta_a   = tf([MAV.a_phi2],[1,MAV.a_phi1,0]);% t
    T_chi_phi       = tf([MAV.gravity/MAV.Va_trim],[1,0]); % t
    T_theta_delta_e = tf(MAV.a_theta3,[1,MAV.a_theta1,MAV.a_theta2]); %t
    T_h_theta       = tf([MAV.Va_trim],[1,0]); % t
    T_h_Va          = tf([MAV.theta_trim],[1,0]); %t
    T_Va_delta_t    = tf([MAV.a_V2],[1,MAV.a_V1]);
    T_Va_theta      = tf([-MAV.a_V3],[1,MAV.a_V1]);
    T_v_delta_r     = tf([MAV.a_beta2],[1,MAV.a_beta1]); % t
