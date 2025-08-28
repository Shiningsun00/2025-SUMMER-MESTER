function [out, ctrl_error] = ctrl_lon2(aircraft, xu_eq, rho, grav, q, r, wc, FreeVar_pnt, Trans_pnt)

ctrl_error = 0; % Initialize controller design error flag to false

Q = diag(q);
fprintf("Qsize: %d\n", length(Q));
R = diag(r);
fprintf("Rsize: %d\n", length(R));

% State space dynamics in the control frame
xeq = xu_eq(1:8); % Pull out the trim condition state variables

ueq = [xu_eq(9:end)]; % Don't add flap input (not used to trim)

NS = 4; % Specify number of aero control surfaces
NP = 9; % Specify number of rotor/propeller control effectors

% Obtain the full longitudinal (linearized) state-space matrices and
% full trim vector
[Alon, Blon, Clon, Dlon, XU0, A_full, B_full, C_full, D_full] = get_long_dynamics_heading(aircraft, xeq, ueq, NS, NP, rho, grav, FreeVar_pnt, Trans_pnt);

if xeq(1) > Trans_pnt(2) % Zero out the lifting rotors after the trans regime ends -> final transition velocity와 xeq상태의 velocity 비교, final transition velocity 넘으면 rotor effector값을 0으로 초기화
    Blon(:,1:8) = zeros(4,8);
end

% size definitions
Nx  = 4;  % system states, x의 size [u,w,q,th]
Ni  = 2;  % integrator states
Nr  = 2;  % reference 
Nu  = 11; % physical controls, [T1 . . . T9, delf, dele]
Nv  = 0;  % virtual controls
Nmu = 4;  % general inputs, mu인데 mu 사용 안함
Nxi = 4;  % general states, xbar인데 사용 안함

Av = Alon;
Bv = Blon;
Cv = Clon([1 2],:);
Dv = Dlon([1 2], :);

At = [ zeros(Ni,Ni)   Cv   ; % A~
       zeros(Nx,Ni)   Av  ]; 
Bt = [ Dv; Bv]; % B~

eigA = eig(At); % A, B가 controllable 한지 판단
for i = 1:length(eigA)
    lam = eigA(i);
    if real(lam) >= 0
        M = [lam*eye(size(At))-At, Bt];
        if rank(M) < size(At, 1)
            fprintf("Stabilizable하지 않은 고유값: %.3f\n", lam)
        end
    end
end
C0 = ctrb(At,Bt);
rank(C0) < size(At,1)

[Kc, P, CLP] = lqr(At,Bt,Q,R); % 폐루프 시스템의 극점 열백터
Ki0 = Kc(:,1:Ni);
Kx0 = Kc(:,Ni+1:Ni+Nxi);

% Control Allocation design 
% Bu = [u u_v]
Bu = zeros(Ni, (Nu+Nv)); % 안씀 

% Define the mapping from general input to 
% physical and virtual control effectors
M = 0; % M = Wc\Bu'*inv(Bu*inv(Wc)*Bu');

A = Alon;
B = Blon;

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

F = [1 0; 0 1; 0 0; 0 0]; % r과 x 같은 component끼리 묶어야하므로, r = [ubar wbar]' -> [ubar wbar 0 0]'
G = zeros(4); % 차원 맞춰야함

% eigenvalue 판단 코드 제외. ctrl_lon 참고
Wc = zeros((Nu+Nv), (Nu+Nv));

% Assign outputs
out.Ap = A;
out.Bp = B;
out.Cp = eye(Nx);
out.Dp = zeros(Nx,Nu);

% out.Ac = zeros(Ni, Ni); % Kv*Mv*Ki
% out.Bc = zeros(Ni, Nx); % Kv*Mv*Kx+Kv*Cv+C
% out.Br = zeros(Ni,Nr);
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

% Output the full linearized dynamics state-space matrices
out.A_full = A_full;
out.B_full = B_full;
out.C_full = C_full;
out.D_full = D_full;

out.XU0 = XU0;
