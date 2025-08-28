function [out, ctrl_error] = ctrl_lat2(aircraft, xu_eq, rho, grav, q, r, wc, FreeVar_pnt, Trans_pnt)

ctrl_error = 0; % Initialize controller design error flag to false

Q = diag(q);
R = diag(r);


% State space dynamics in the control frame
xeq = xu_eq(1:8); % Pull out the trim condition state variables

ueq = [xu_eq(9:end)]; % Pull out the trim condition effector variables

NS = 4; % Specify number of aero control surfaces
NP = 9; % Specify number of rotor/propeller control effectors

% Obtain the full longitudinal (linearized) state-space matrices and
% full trim vector
[Alat, Blat, Clat, Dlat, XU0] = get_lat_dynamics_heading(aircraft, xeq, ueq, NS, NP, rho, grav, FreeVar_pnt, Trans_pnt);

if xeq(1) > Trans_pnt(2) % Zero out the lifting rotors after the trans regime ends -> final transition velocity와 xeq상태의 velocity 비교, final transition velocity 넘으면 rotor effector값을 0으로 초기화
    Blon(:,1:8) = zeros(4,8);
end

% size definitions
Nx  = 4;  % system states, x의 size [u,w,q,th]
Ni  = 2;  % integrator states
Nr  = 2;  % reference 
Nu  = 10; % physical controls, [T1 . . . T8, delf, dele], lateral에서 pusher 사용 안함
Nv  = 0;  % virtual controls
Nmu = 4;  % general inputs, mu인데 mu 사용 안함
Nxi = 4;  % general states, xbar인데 사용 안함

Av = Alat;
Bv = Blat;
Cv = Clat([1 3], :);
Dv = Dlat([1 3], :);

At = [ zeros(Ni,Ni)   Cv   ; % A~
       zeros(Nx,Ni)   Av  ]; 
Bt = [ Dv; Bv]; % B~



[Kc, P, CLP] = lqr(At,Bt,Q,R); % 폐루프 시스템의 극점 열백터
Ki0 = Kc(:,1:Ni);
Kx0 = Kc(:,Ni+1:Ni+Nxi);

% Control Allocation design 
% Bu = [u u_v]
Bu = zeros(Ni, (Nu+Nv)); % 안씀 

% Define the mapping from general input to 
% physical and virtual control effectors
M = 0; % M = Wc\Bu'*inv(Bu*inv(Wc)*Bu');

A = Alat;
B = Blat;

% Tracking states
C = Cv;
D = Dv;

% add a column of zeros into the 
% state feedback matrix so we can add
% theta back in as a state
% virtual control 없애므로 그냥 사용 가능
Kx = Kx0;
Ki = Ki0;

% Break up the Allocation Matrix, 안쓰니까 0
Mu = 0;
Mv = 0;

% Virtual control, 안쓰니까 0
Cv = zeros(Nv, Nx);
Kv = zeros(Ni, Nv);

% 
th0  = XU0(11);
phi0 = XU0(10);
q0   = XU0(5);
vc0  = XU0(1:3);
vb0  = Rx(phi0)*Ry(th0)*vc0;
ub0  = vb0(1);

F = [1 0; 0 0; 0 1; 0 0]; % r과 x 같은 component끼리 묶어야하므로, r = [vbar 0 psi_dot 0]'
G = zeros(4); % 차원 맞춰야함 & 아마 virtual control로 인해 생겨난 항인 것 같은데 우선 제외. G = [0 ub0; 0 0; 0 0];

% eigenvalue 판단 코드 제외. ctrl_lon 참고
Wc = zeros((Nu+Nv), (Nu+Nv));

% Assign outputs
out.Ap = A;
out.Bp = B;
out.Cp = eye(Nx);
out.Dp = zeros(Nx,Nu);

% out.Ac = zeros(Ni, Ni); % Kv*Mv*Ki
% out.Bc = zeros(Ni, Nx); % Kv*Mv*Kx+Kv*Cv+C
% out.Br = zeros(Ni, Nr);
% 
% out.Cc = zeros(Nu, Ni); % -Mu*Ki
% out.Dc = zeros(Nu, Nx); % -Mu*Kx
% out.Dr = zeros(Nu, Nr);

out.Ki = Ki;
out.Kx = Kx;
% out.Kv = Kv;

% out.F  = F;
% out.G  = G;
out.C  = C;
% out.Cv = Cv;

out.Q = Q;
out.R = R;

out.W  = Wc;
out.B  = Bu;

out.XU0 = XU0;
% 구조체로 output이 나감. 결과적으로 output은 하나라고 봐도 됨.


