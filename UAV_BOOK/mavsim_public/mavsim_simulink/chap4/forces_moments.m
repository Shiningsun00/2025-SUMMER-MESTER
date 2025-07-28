% forces_moments.m
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

function out = forces_moments(x, delta, wind, P)
    
    %add parameters
    addpath('../parameters');
    aerosonde_parameters
    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    %% compute wind data in NED

    % define transformation matrix
    R_roll = [...
            1, 0, 0;...
            0, cos(phi), -sin(phi);...
            0, sin(phi), cos(phi)];
    R_pitch = [...
            cos(theta), 0, sin(theta);...
            0, 1, 0;...
            -sin(theta), 0, cos(theta)];
    R_yaw = [...
            cos(psi), -sin(psi), 0;...
            sin(psi), cos(psi), 0;...
            0, 0, 1];
    R = R_roll*R_pitch*R_yaw;

    % define steady(w_s), gust(w_g) wind vector
    w_s = [w_ns;w_es;w_ds];
    w_g = [u_wg;v_wg;w_wg];
    % define wind velocity(v_w) in the body frame
    v_w = R*w_s + w_g;
    w_n = v_w(1);
    w_e = v_w(2);
    w_d = v_w(3);
    
    %% compute air data

    % define ground velocity(v_g)
    v_g = [u;v;w];
    % define air speed vector(v_a)
    v_a = v_g - v_w;
    u_r = v_a(1);
    v_r = v_a(2);
    w_r = v_a(3);

    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/Va);
    
    %% compute external forces and torques on aircraft

    % define coefficients
    C_D_a = MAV.C_D_0 + MAV.C_D_alpha * alpha; % 조건에 따라 비선형 방정식 사용 가능
    C_L_a = MAV.C_L_0 + MAV.C_L_alpha * alpha; % 조건에 따라 비선형 방정식 사용 가능
    C_X_a = -C_D_a * cos(alpha) + C_L_a * sin(alpha);
    C_X_q_a = -MAV.C_D_q * cos(alpha) + MAV.C_L_q * sin(alpha);
    C_X_delta_e_a = -MAV.C_D_delta_e * cos(alpha) + MAV.C_L_delta_e * sin(alpha);
    C_Z_a = -C_D_a * sin(alpha) - C_L_a * cos(alpha);
    C_Z_q_a = -MAV.C_D_q * sin(alpha) - MAV.C_L_q * cos(alpha);
    C_Z_delta_e_a = -MAV.C_D_delta_e * sin(alpha) - MAV.C_L_delta_e * cos(alpha);

    V_in = MAV.V_max * delta_t;
    a = (MAV.rho * MAV.C_Q0 * MAV.D_prop^5)/((2*pi)^2);
    b = (MAV.rho * MAV.C_Q1 * Va * MAV.D_prop^4)/(2*pi) + MAV.KQ * MAV.K_V/MAV.R_motor;
    c = MAV.rho * MAV.D_prop^3 * MAV.C_Q2 * Va^2 - MAV.KQ*V_in/MAV.R_motor + MAV.KQ*MAV.i0;
    omega_p = (-b + sqrt(b^2 - 4*a*c))/(2*a);
    T_p = ((MAV.rho*MAV.D_prop^4*MAV.C_T0)/(4*pi^2))*omega_p^2 + ((MAV.rho*MAV.D_prop^3*MAV.C_T1*Va)/(2*pi))*omega_p + MAV.rho*MAV.D_prop^2*MAV.C_T2*Va^2;
    Q_p = ((MAV.rho*MAV.D_prop^5*MAV.C_Q0)/(4*pi^2))*omega_p^2 + ((MAV.rho*MAV.D_prop^4*MAV.C_Q1*Va)/(2*pi))*omega_p + MAV.rho*MAV.D_prop^3*MAV.C_Q2*Va^2;

    Force(1) =  -MAV.mass*MAV.gravity*sin(theta) + T_p + 0.5*MAV.rho*Va^2*MAV.S_wing*(C_X_a + C_X_q_a*MAV.c*q/(2*Va) + C_X_delta_e_a*delta_e);
    Force(2) =  MAV.mass*MAV.gravity*cos(theta)*sin(phi) + 0.5*MAV.rho*Va^2*MAV.S_wing*(MAV.C_Y_0 + MAV.C_Y_beta*beta + MAV.C_Y_p*MAV.b*p/(2*Va) + MAV.C_Y_r*MAV.b*r/(2*Va) + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);
    Force(3) =  MAV.mass*MAV.gravity*cos(theta)*cos(phi) + 0.5*MAV.rho*Va^2*MAV.S_wing*(C_Z_a + C_Z_q_a*MAV.c*q/(2*Va) + C_Z_delta_e_a*delta_e);
    
    Torque(1) = -Q_p + 0.5*MAV.rho*Va^2*MAV.S_wing*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta*beta + MAV.C_ell_p*MAV.b*p/(2*Va) + MAV.C_ell_r*MAV.b*r/(2*Va) + MAV.C_ell_delta_a*delta_a + MAV.C_ell_delta_r*delta_r);
    Torque(2) = 0.5*MAV.rho*Va^2*MAV.S_wing*MAV.c*(MAV.C_m_0 + MAV.C_m_alpha*alpha + MAV.C_m_q*MAV.c*q/(2*Va) + MAV.C_m_delta_e*delta_e);   
    Torque(3) = 0.5*MAV.rho*Va^2*MAV.S_wing*MAV.b*(MAV.C_n_0 + MAV.C_n_beta*beta + MAV.C_n_p*MAV.b*p/(2*Va) + MAV.C_n_r*MAV.b*r/(2*Va) + MAV.C_n_delta_a*delta_a + MAV.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



