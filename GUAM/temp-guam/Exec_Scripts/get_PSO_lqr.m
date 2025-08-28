function [Ki0, Kx0] = get_PSO_lqr(At,Bt,Q,R)

coder.extrinsic('lqr');


Nx  = 4;  % system states, x의 size [u,w,q,th]
Ni  = 2;  % integrator states
Nr  = 2;  % reference 
Nu  = 11; % physical controls, [T1 . . . T9, delf, dele]
Nv  = 0;  % virtual controls
Nmu = 4;  % general inputs, mu인데 mu 사용 안함
Nxi = 4;  % general states, xbar인데 사용 안함

Kc = lqr(At,Bt,Q,R); % 폐루프 시스템의 극점 열백터
Ki0 = Kc(Nu,1:Ni);
Kx0 = Kc(Nu,Ni+1:Ni+Nxi);
