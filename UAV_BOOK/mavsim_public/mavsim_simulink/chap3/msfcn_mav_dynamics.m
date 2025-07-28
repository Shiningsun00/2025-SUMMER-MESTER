function msfcn_mav_dynamics(block)
% Level-2 MATLAB file S-Function for MAV dynamics

  setup(block);

%endfunction

function setup(block)
    
    % Register number of dialog parameters   
    block.NumDialogPrms  = 1; % MAV struct
    
    % Register number of input and output ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    % Setup functional port properties to dynamically inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input: 6 forces/moments
    block.InputPort(1).Dimensions        = 6;
    block.InputPort(1).DirectFeedthrough = false;

    % Output: 12 outputs (pos, vel, euler or quat, rates)
    block.OutputPort(1).Dimensions       = 12;

    % Number of continuous states
    block.NumContStates = 13;

    % Sample times
    block.SampleTimes = [0 0]; % continuous

    % Block methods
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);
    
%endfunction

function InitializeConditions(block)
    MAV = block.DialogPrm(1).Data;

    block.ContStates.Data = [...
        MAV.pn0; MAV.pe0; MAV.pd0; ...
        MAV.u0; MAV.v0; MAV.w0; ...
        MAV.e0; MAV.e1; MAV.e2; MAV.e3; ...
        MAV.p0; MAV.q0; MAV.r0];

%endfunction

function Derivatives(block)
    MAV = block.DialogPrm(1).Data;
    
    x  = block.ContStates.Data;
    uu = block.InputPort(1).Data;

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    e0    = x(7);
    e1    = x(8);
    e2    = x(9);
    e3    = x(10);
    p     = x(11);
    q     = x(12);
    r     = x(13);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    pndot = u*(e0^2+e1^2-e2^2-e3^2) + v*2*(e1*e2-e0*e3) + w*2*(e1*e3+e0*e2);
    
    pedot = u*2*(e1*e2+e0*e3) + v*(e0^2-e1^2+e2^2-e3^2) + w*2*(e2*e3-e0*e1);
    
    pddot = u*2*(e1*e3-e0*e2) + v*2*(e2*e3+e0*e1) + w*(e0^2-e1^2-e2^2+e3^2);
    
    udot = r*v-q*w + fx/MAV.mass;
    
    vdot = p*w-r*u + fy/MAV.mass;
    
    wdot = q*u-p*v + fz/MAV.mass;
       
    e0dot = 0.5*(e1*(-p)+e2*(-q)+e3*(-r));
    e1dot = 0.5*(e0*p+e2*r+e3*(-q));
    e2dot = 0.5*(e0*q+e1*(-r)+e3*(p));
    e3dot = 0.5*(e0*r+e1*q+e2*(-p));
        
    pdot = MAV.Gamma1*p*q-MAV.Gamma2*q*r+MAV.Gamma3*ell+MAV.Gamma4*n;
    qdot = MAV.Gamma5*p*r-MAV.Gamma6*(p^2-r^2)+m/MAV.Jy;
    rdot = MAV.Gamma7*p*q-MAV.Gamma1*q*r+MAV.Gamma4*ell+MAV.Gamma8*n;
        
    block.Derivatives.Data = [pndot; pedot; pddot; udot; vdot; wdot;
                              e0dot; e1dot; e2dot; e3dot; pdot; qdot; rdot];


%endfunction

function Outputs(block)
    x = block.ContStates.Data;
      % quaternion 추출
    e0 = x(7);
    e1 = x(8);
    e2 = x(9);
    e3 = x(10);
    quaternion = [e0, e1, e2, e3];
    [phi, theta, psi] = Quaternion2Euler(quaternion);

    y = [
        x(1); x(2); x(3);   % position pn, pe, pd
        x(4); x(5); x(6);   % velocity u, v, w
        phi; theta; psi;    % euler angles
        x(11); x(12); x(13) % angular rates p, q, r
    ];
    block.OutputPort(1).Data = y;

