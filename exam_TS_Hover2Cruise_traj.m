%% Hover2Cruise trajectory
% August, 2025 by haein Jeon
% Using 3D missile guidance to generate trajectory + Attitude injection

%% Hover2Cruise trajectory
% August, 2025 by haein Jeon
% Using 3D missile guidance to generate trajectory + Attitude injection
% - Plan A: BusSelector에서 phi/theta 명령 출력 포트에 주입
% - Plan B: 실패 시, 신호 라벨(roll/pitch/phi/theta + cmd/des/ref/command)로 라인 스캔 후 주입

%% Hover2Cruise trajectory + Ref injection (chi_des / Vel_bIc_des / chi_dot_des)
% One-shot script by ChatGPT (합본)
clear; clc;

%% Toggles
USE_MY_CMD       = 1;   % 1: 아래 My_* 레퍼런스 사용, 0: 모델 원래 신호 사용
USE_TEST_STEP_CHI= 0;   % 1: chi_des에 1.0~1.2s +20deg 스텝 강제 주입 (반응 체크용)
model            = 'GUAM';

%% --- Model / Input mode ---
userStruct.variants.refInputType = 3;   % RefInputEnum.TIMESERIES

%% Initial condition / target
missile.V = 8;  % m/s
missile.x = 0;  missile.y = 0;  missile.z = 0;   % NED (-z up)
tgt.x = 150; tgt.y = 0; tgt.z = -100;

% Initial LOS & LOS angles
RTP   = [(tgt.x - missile.x), (tgt.y - missile.y), (tgt.z - missile.z)];
R     = norm(RTP);
thetaL = asin(max(-1,min(1,RTP(3)/R)));   % LOS elev
psiL   = atan2(RTP(2), RTP(1));           % LOS az
thetaM = deg2rad(10);                     % missile elev (LOS frame)
psiM   = deg2rad(10);                     % missile az (LOS frame)

% Initial velocity via LOS triad
e_r     = [cos(thetaL)*cos(psiL);  cos(thetaL)*sin(psiL);  sin(thetaL)];
e_psi   = [-sin(psiL);             cos(psiL);              0          ];
e_theta = [-sin(thetaL)*cos(psiL); -sin(thetaL)*sin(psiL); cos(thetaL)];
uL = [cos(thetaM)*cos(psiM);  cos(thetaM)*sin(psiM);  sin(thetaM)];
uI = e_r*uL(1) + e_psi*uL(2) + e_theta*uL(3);
vI = missile.V * uI;
missile.xV = vI(1); missile.yV = vI(2); missile.zV = vI(3);

% gains
c1 = 5; c2 = 5;

% desired acceleration in LOS frame (a_psi, a_theta)
accelY = -(missile.V)^2 / R * sin(psiM) ...
       + (missile.V)^2 / (4*R) * sin(2*thetaM) * tan(thetaL) * sin(2*psiM) ...
       - c2 * (missile.V)^2 / R * cos(thetaM) * sin(psiM/4) * cos(psiM/4);
accelZ = -(missile.V)^2 / R * sin(thetaM) * cos(psiM) ...
       - (missile.V)^2 / R * cos(thetaM) * tan(thetaL) * (sin(psiM))^2 ...
       - c1 * (missile.V)^2 / R * sin(thetaM/4) * cos(thetaM/4);

%% Simulation set
dt        = 1e-4;
Smax      = 100;
Niter     = max(2, round(Smax/dt));
accel_max = 10 * 9.81;
tol_R     = 0.01;
g         = 9.81;

%% Preallocate logs
t  = nan(Niter,1);
mx = nan(Niter,1);  my = nan(Niter,1);  mz = nan(Niter,1);
mxV= nan(Niter,1);  myV= nan(Niter,1);  mzV= nan(Niter,1);
thM= nan(Niter,1);  thL= nan(Niter,1);  psM= nan(Niter,1);  psL= nan(Niter,1);
aN_log = nan(Niter,1); aE_log = nan(Niter,1); aD_log = nan(Niter,1);
phi_cmd_log = nan(Niter,1); theta_cmd_log = nan(Niter,1); psi_cmd_log = nan(Niter,1);

%% Main loop
k_end = Niter;
for k = 1:Niter
    t(k) = (k-1)*dt;

    RTP = [(tgt.x - missile.x), (tgt.y - missile.y), (tgt.z - missile.z)];
    R   = norm(RTP);

    m = max(-1, min(1, RTP(3)/R));
    thetaL = asin(m);
    thetaL = max(-pi/2+1e-6, min(pi/2-1e-6, thetaL));
    psiL   = atan2(RTP(2), RTP(1));

    % LOS triad update
    e_r     = [cos(thetaL)*cos(psiL);  cos(thetaL)*sin(psiL);  sin(thetaL)];
    e_psi   = [-sin(psiL);             cos(psiL);              0          ];
    e_theta = [-sin(thetaL)*cos(psiL); -sin(thetaL)*sin(psiL); cos(thetaL)];

    % angle rates
    thetaM_dot = (accelZ)/(missile.V) ...
               + missile.V/R * cos(thetaM) * tan(thetaL) * (sin(psiM))^2 ...
               + missile.V/R * sin(thetaM) * cos(psiM);
    psiM_dot = (accelY)/(missile.V * cos(thetaM)) ...
             - (missile.V)/R * sin(thetaM) * tan(thetaL) * sin(psiM) * cos(psiM) ...
             + missile.V / (R * cos(thetaM)) * (sin(thetaM))^2 * sin(psiM) ...
             + missile.V / R * cos(thetaM) * sin(psiM);

    thetaM = thetaM + thetaM_dot*dt;
    psiM   = psiM   + psiM_dot*dt;

    % re-calc accel
    accelY = -(missile.V)^2 / R * sin(psiM) ...
           + (missile.V)^2 / (4*R) * sin(2*thetaM) * tan(thetaL) * sin(2*psiM) ...
           - c2 * (missile.V)^2 / R * cos(thetaM) * sin(psiM/4) * cos(psiM/4);
    accelZ = -(missile.V)^2 / R * sin(thetaM) * cos(psiM) ...
           - (missile.V)^2 / R * cos(thetaM) * tan(thetaL) * (sin(psiM))^2 ...
           - c1 * (missile.V)^2 / R * sin(thetaM/4) * cos(thetaM/4);

    % saturation
    a_norm = hypot(accelY, accelZ);
    if a_norm > accel_max
        s = accel_max / a_norm;
        accelY = accelY * s; accelZ = accelZ * s;
    end

    % LOS accel -> NED accel (a_r=0)
    aNED = accelY*e_psi + accelZ*e_theta;
    aN = aNED(1); aE = aNED(2); aD = aNED(3);
    aN_log(k)=aN; aE_log(k)=aE; aD_log(k)=aD;

    % accel -> attitude (for logging only)
    psi_acc = atan2(aE, aN);
    denom = max(0.5, g - aD);
    phi_cmd   = atan2(hypot(aN,aE), denom);
    theta_cmd = atan2( aN*cos(psi_acc) + aE*sin(psi_acc), denom );
    psi_cmd   = psi_acc;
    phi_cmd_log(k)=phi_cmd; theta_cmd_log(k)=theta_cmd; psi_cmd_log(k)=psi_cmd;

    % velocity update (constant speed)
    uL = [cos(thetaM)*cos(psiM);  cos(thetaM)*sin(psiM);  sin(thetaM)];
    uI = e_r*uL(1) + e_psi*uL(2) + e_theta*uL(3);
    vI = missile.V * uI;
    missile.xV = vI(1); missile.yV = vI(2); missile.zV = vI(3);

    % integrate pos
    missile.x = missile.x + missile.xV*dt;
    missile.y = missile.y + missile.yV*dt;
    missile.z = missile.z + missile.zV*dt;

    mx(k)=missile.x; my(k)=missile.y; mz(k)=missile.z;
    mxV(k)=missile.xV; myV(k)=missile.yV; mzV(k)=missile.zV;
    thM(k)=thetaM; thL(k)=thetaL; psM(k)=psiM; psL(k)=psiL;

    if R <= tol_R
        fprintf('Interception at t = %.6f s (k=%d, R=%.4g m)\n', t(k), k, R);
        k_end = k; break;
    end
end

%% Trim logs
valid = isfinite(t);
if any(valid), last = find(valid,1,'last'); else, last = 0; end
k_end = min(k_end, max(2,last));
t      = t(1:k_end);
pos    = [mx(1:k_end),  my(1:k_end),  mz(1:k_end)];
vel_i  = [mxV(1:k_end), myV(1:k_end), mzV(1:k_end)];

if numel(t) < 2
    t     = [0; dt];
    pos   = [pos(1,:);   pos(1,:)];
    vel_i = [vel_i(1,:); vel_i(1,:)];
end

%% chi / chi_dot
chi  = unwrap(atan2(vel_i(:,2), vel_i(:,1)));
chid = [0; diff(chi)./diff(t)];
if numel(chid) ~= numel(t), chid(end+1) = chid(end); end

%% Build Ref timeseries
PhiCmd_ts   = timeseries(phi_cmd_log(1:k_end),   t);   % (로그용/미사용)
ThetaCmd_ts = timeseries(theta_cmd_log(1:k_end), t);   % (로그용/미사용)
PsiCmd_ts   = timeseries(psi_cmd_log(1:k_end),   t);   % (로그용/미사용)
aCmdNED_ts  = timeseries([aN_log(1:k_end), aE_log(1:k_end), aD_log(1:k_end)], t);

vel = vel_i;  % heading-frame = inertial 취급
RefInput.Vel_bIc_des = timeseries(vel,   t);
RefInput.pos_des     = timeseries(pos,   t);
RefInput.chi_des     = timeseries(chi,   t);
RefInput.chi_dot_des = timeseries(chid,  t);
RefInput.vel_des     = timeseries(vel_i, t);

% === 우리가 "주입"할 신호들 ===
My_chi_des     = RefInput.chi_des;        % 1x1
My_chi_dot_des = RefInput.chi_dot_des;    % 1x1
My_Vel_bIc_des = RefInput.Vel_bIc_des;    % Nx3

% 테스트: chi_des 스텝
if USE_TEST_STEP_CHI
    tmp = My_chi_des.Data;
    tmp(t>=1 & t<1.2) = tmp(t>=1 & t<1.2) + deg2rad(20);
    My_chi_des = timeseries(tmp, t);
end

%% Push to base WS
clear target; target.RefInput = RefInput;
if ~bdIsLoaded(model), load_system(model); end
set_param(model,'FastRestart','off');

assignin('base','USE_MY_CMD',       double(USE_MY_CMD));
assignin('base','target',           target);
assignin('base','userStruct',       userStruct);
assignin('base','My_chi_des',       My_chi_des);
assignin('base','My_chi_dot_des',   My_chi_dot_des);
assignin('base','My_Vel_bIc_des',   My_Vel_bIc_des);

%% Compile (resolve variants)
try, set_param(model,'SimulationCommand','update'); catch, end

%% === Inject at chi_des / Vel_bIc_des / chi_dot_des ===
% 네 디버그에서 나온 정확한 후보 3곳을 우선 시도:
% 1) .../Baseline/Convert Velocity and Position Error To Control Frame/Bus Selector2: chi_des,pos_des
% 2) .../Baseline/Perturbation Variables for Linear Control/Bus Selector2: Vel_bIc_des,chi_dot_des
% 3) .../BASELINE/Bus Selector7: Vel_bIc_des,chi_des

cands = {
 'Vehicle Generalized Control/Lift+Cruise Control/BASELINE/Baseline/Convert Velocity and Position Error To Control Frame/Bus Selector2'
 'Vehicle Generalized Control/Lift+Cruise Control/BASELINE/Baseline/Perturbation Variables for Linear Control/Bus Selector2'
 'Vehicle Generalized Control/Lift+Cruise Control/BASELINE/Bus Selector7'
};

% 각각에서 출력 이름을 읽어 필요한 신호만 주입
needList = { 'chi_des', 'Vel_bIc_des', 'chi_dot_des' };
varMap.chi_des     = 'My_chi_des';
varMap.chi_dot_des = 'My_chi_dot_des';
varMap.Vel_bIc_des = 'My_Vel_bIc_des';

did = false;
for ci = 1:numel(cands)
    bsFull = [model '/Vehicle Simulation/' cands{ci}];  % 절대경로로 조합
    if ~exist_block(bsFull), continue; end
    try
        ph  = get_param(bsFull,'PortHandles');
        sig = strtrim(get_param(bsFull,'OutputSignals'));
        toks = regexp(sig,'\s*,\s*','split'); toks = toks(~cellfun('isempty',toks));
        for ksig = 1:numel(needList)
            key = needList{ksig};
            idx = find(strcmp(toks, key), 1, 'first');
            if ~isempty(idx)
                fromVar = varMap.(key);
                inject_switch_on_selector_output(bsFull, idx, fromVar, 'USE_MY_CMD', ['UseMy_' key]); 
                fprintf('[inject] %s -> %s (on %s / port #%d)\n', key, fromVar, bsFull, idx);
                did = true;
            end
        end
    catch ME
        warning('주입 시도 실패(%s): %s', bsFull, ME.message);
    end
end

if ~did
    error('주입 실패: chi_des / Vel_bIc_des / chi_dot_des 에 해당하는 출력 포트를 못 찾았습니다.');
end

%% Sim & quick plot
set_param(model,'SimulationMode','accelerator','StopTime', sprintf('%.6f', t(end)), ...
                 'SaveTime','off','SaveState','off','SaveOutput','off', ...
                 'ReturnWorkspaceOutputs','off','SignalLogging','off');
try
    SimIn = simSetup(); simOut = sim(SimIn);
catch
    try simSetup; end %#ok<TRYNC>
    simOut = sim(model);
end

try
    figure('Name','Injected references');
    subplot(3,1,1); plot(My_chi_des.Time, My_chi_des.Data); grid on; ylabel('\chi_{des} (rad)'); title('Injected references');
    subplot(3,1,2); plot(My_chi_dot_des.Time, My_chi_dot_des.Data); grid on; ylabel('\dot{\chi}_{des} (rad/s)');
    subplot(3,1,3); plot(My_Vel_bIc_des.Time, My_Vel_bIc_des.Data); grid on; ylabel('Vel\_bIc\_des (m/s)'); xlabel('t (s)');
catch
end

%% ============= Local helpers =============
function inject_switch_on_selector_output(bsPath,outIdx,fromVar,useVar,tag)
    parent = get_param(bsPath,'Parent');
    ph  = get_param(bsPath,'PortHandles');  op  = ph.Outport(outIdx);
    ln  = get_param(op,'Line');  if ln==-1, error('Outport(%d)에 라인이 없습니다.', outIdx); end
    dstPorts = get_param(ln,'DstPortHandle');
    delete_line(ln);

    pos = get_param(bsPath,'Position'); y0 = pos(2)+40+70*outIdx;
    sw = [parent '/' tag '_SW'];  fw = [parent '/' tag '_FW'];  cv = [parent '/' tag '_CONST'];
    if exist_block(sw), delete_block(sw); end
    if exist_block(fw), delete_block(fw); end
    if ~exist_block(cv)
        add_block('simulink/Sources/Constant', cv, ...
                  'Value', useVar, 'Position',[pos(1)+60 y0+30 pos(1)+90 y0+50]);
    end
    add_block('simulink/Sources/From Workspace', fw, ...
              'VariableName', fromVar, 'Interpolate','on','SampleTime','-1', ...
              'Position',[pos(1)+60 y0 pos(1)+90 y0+20]);
    add_block('simulink/Signal Routing/Switch', sw, ...
              'Criteria','u2 >= Threshold','Threshold','0.5', ...
              'Position',[pos(1)+120 y0 pos(1)+180 y0+40]);

    % Selector out -> Switch u3 (기존), FromWS -> u1(내 것), Const -> u2(조건)
    add_line(parent, [get_param(bsPath,'Name') '/' num2str(outIdx)], [get_param(sw,'Name') '/3'],'autorouting','on');
    add_line(parent, [get_param(fw,'Name') '/1'],                   [get_param(sw,'Name') '/1'],'autorouting','on');
    add_line(parent, [get_param(cv,'Name') '/1'],                   [get_param(sw,'Name') '/2'],'autorouting','on');

    % Switch out -> 원래 목적지들
    if ~iscell(dstPorts), dstPorts={dstPorts}; end
    for i=1:numel(dstPorts)
        dp = dstPorts{i};
        add_line(parent, [get_param(sw,'Name') '/1'], ...
            [get_param(get_param(dp,'Parent'),'Name') '/' num2str(get_param(dp,'PortNumber'))], 'autorouting','on');
    end
end

function tf = exist_block(path)
    try get_param(path,'Handle'); tf=true; catch, tf=false; end
end


% % prescibe inertial position (NED)
% pos = [0 0 -100]; % Inertial Positions (x,y, -z) row vector for each time


% %% Solution
% % setting
% pos = [1]
% while ~ok
%     d.p = inpu
%     t('Enter desired position as [x z]: ');
%     ok = isnumeric(d.p) && numel(d.p)==2 && all(isfinite(d.p));
%     if ~ok, fprintf('>> Please enter a numeric 3-element vector, e.g., [10 2]\n'); end
% end
% d.p = d.p(:).';     % row vector
% target.x = d.p(1);
% target.y = 0;
% target.z = d.p(2);
% target.V = 0;
% target.HDG =0;
% target.GAM = 0;
% target.xV = target.V * cos(target.GAM);
% target.yV = 0;
% target.zV = target.V * sin(target.GAM);
% 
% % Missile
% missile.x = 0;
% missile.y = 0;
% missile.z = 0;
% missile.V = 300;
% missile.GAM = deg2rad(45);
% missile.HDG = 0;
% missile.xV = missile.V * cos(missile.GAM)*cos(missile.HDG);
% missile.yV = missile.V * cos(missile.GAM)*sin(missile.HDG);
% missile.zV = missile.V * sin(missile.GAM);
% 
% % Navigation constant
% N = 3;
% 
% %% Parameters
% accel_max = 180;       % Max acceleration
% dt = 0.0001;            % Time step
% S = 100;                % Simulation time
% Niter = S/dt;           % Number of iterations
% interception_range = 1;
% 
% %% Preallocate Logger
% logger.t = nan(1, Niter);
% logger.mx = nan(1, Niter);
% logger.my = nan(1, Niter);
% logger.mz = nan(1, Niter);
% logger.mxV = nan(1, Niter);
% logger.myV = nan(1, Niter);
% logger.mzV = nan(1, Niter);
% logger.m.GAM = nan(1, Niter);
% logger.tx = nan(1, Niter);
% logger.ty = nan(1, Niter);
% logger.tz = nan(1, Niter);
% logger.txV = nan(1, Niter);
% logger.tyV = nan(1, Niter);
% logger.tzV = nan(1, Niter);
% logger.R = nan(1, Niter);
% logger.LOS = nan(1, Niter);
% logger.LOSrate = nan(1, Niter);
% logger.accel_m = nan(1, Niter);
% 
% %% Initial propagation
% missile.x = missile.x + missile.xV * dt;
% missile.z = missile.z + missile.zV * dt;
% target.x = target.x + target.xV * dt;
% target.z = target.z + target.zV * dt;
% 
% %% Simulation loop
% for k = 1:Niter
% 
%     
%     RTP = [target.x - missile.x, target.z - missile.z];
%     R = norm(RTP);
%     LOS = atan2(RTP(2), RTP(1));
%     LOSrate = (target.V * sin(target.GAM - LOS)-missile.V*sin(missile.GAM - LOS)) / R;
% 
%     % --- Guidance law ---
%     accel_m = N * missile.V * LOSrate;
% 
%     % --- Acceleration saturation ---
%     accel_m = max(min(accel_m, accel_max), -accel_max);
% 
%     % --- Interception check ---
%     if R <= interception_range
%         disp(['Interception occurred at t = ', num2str(k*dt), ' sec.']);
%         break;
%     end
% 
%     % --- Missile dynamics update ---
%     M_accel_z =  accel_m * cos(missile.GAM);
%     M_accel_x = -accel_m * sin(missile.GAM);
%     missile.xV = missile.xV + M_accel_x * dt;
%     missile.zV = missile.zV + M_accel_z * dt;
%     missile.x  = missile.x  + missile.xV * dt;
%     missile.z  = missile.z  + missile.zV * dt;
%     missile.GAM = atan2(missile.zV, missile.xV);
% 
%     % --- Target motion update (constant velocity) ---
%     target.x = target.x + target.xV * dt;
%     target.z = target.z + target.zV * dt;
% 
%     % ㄴogging ---
%     logger.t(k) = k * dt;
%     logger.mx(k) = missile.x;
%     logger.my(k) = missile.y;
%     logger.mz(k) = missile.z;
%     logger.mxV(k) = missile.xV;
%     logger.myV(k) = missile.yV;
%     logger.mzV(k) = missile.zV;
%     logger.m.GAM(k) = missile.GAM;
%     logger.tx(k) = target.x;
%     logger.ty(k) = target.y;
%     logger.tz(k) = target.z;
%     logger.txV(k) = target.xV;
%     logger.tyV(k) = target.yV;
%     logger.tzV(k) = target.zV;
%     logger.R(k) = R;
%     logger.LOS(k) = LOS;
%     logger.LOSrate(k) = LOSrate;
%     logger.accel_m(k) = accel_m;
% 
% end
% 
% RefInput.Vel_bIc_des    = timeseries(vel,time); % Heading frame velocity
% RefInput.pos_des        = timeseries(pos,time); % Inertial Position
% RefInput.chi_des        = timeseries(chi,time); % Heading Angle
% RefInput.chi_dot_des    = timeseries(chid,time); % Heading Angle Rate
% RefInput.vel_des        = timeseries(vel_i,time); % Inertial Position
% 
% target.RefInput = RefInput;
% 
% 
% %% Animation of Missile and Target Trajectories (3D view, y=0 plane)
% filename = 'PPN_Animation.gif';
% 
% figure; grid on; hold on; axis equal
% xlabel('x (m)','Fontweight','bold','Fontsize',10);
% ylabel('y (m)','Fontweight','bold','Fontsize',10);
% zlabel('z (m)','Fontweight','bold','Fontsize',10);
% title('Missile vs Target Trajectory (PPN, 3D view)', 'Fontweight','bold','Fontsize',14);
% 
% missile_plot = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 2);
% target_plot  = plot3(NaN, NaN, NaN, 'bo', 'LineWidth', 2);
% ax = gca; ax.XAxisLocation='bottom'; ax.YAxisLocation='left';
% ax.LineWidth = 1.5;   % 전체 축선 굵기
% ax.Box = 'off'; 
% legend('Missile', 'Target', 'Location','east');
% 
% % 축 고정 (y는 ±작은 범위)
% pad  = 200;
% ypad = 5;
% 
% xlim([min([logger.mx logger.tx])-pad, max([logger.mx logger.tx])+pad]);
% ylim([-ypad, ypad]);
% zlim([min([logger.mz logger.tz])-pad, max([logger.mz logger.tz])+pad]);
% view(35, 25);
% 
% % 요격 시점
% intercept_frame = find(logger.R <= interception_range, 1, 'first');
% if isempty(intercept_frame), intercept_frame = numel(logger.t); end
% 
% % 애니메이션
% for i = 1:2000:intercept_frame
%     set(missile_plot, 'XData', logger.mx(1:i), 'YData', logger.my(1:i), 'ZData', logger.mz(1:i));
%     set(target_plot,  'XData', logger.tx(1:i), 'YData', logger.ty(1:i), 'ZData', logger.tz(1:i));
%     drawnow;
% 
%     % GIF 저장
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [A,map] = rgb2ind(im,256);
%     if i == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.01);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.01);
%     end
% end
% disp('✅ GIF animation complete: PPN_Animation.gif');
% userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
% 

%% Solution 2
% % 1) 기본 Hover→Cruise 타임시리즈 생성
% exam_TS_Hover2Cruise_traj;  % target.RefInput.* (ft, NED) 생성된다고 가정
% 
% % --- 유틸: timeseries → 배열
% extractTS = @(ts) deal(ts.Time(:), ts.Data);
% 
% % 2) Hover→Cruise 추출
% [t1, pos1]  = extractTS(target.RefInput.pos_des);
% [~,  vel1]  = extractTS(target.RefInput.vel_des);
% [~,  chi1]  = extractTS(target.RefInput.chi_des);
% [~,  chid1] = extractTS(target.RefInput.chi_dot_des);
% 
% % 3) 미사일 세그먼트를 GUAM 형식(NED, ft)으로 생성
% [tm, posm, velm, chim, chimd] = missileSegment_to_GUAM();
% 
% % 4) 끝점 정렬(회전+이동)
% p1_end   = pos1(end,:);                 % [N E D] (ft)
% chi1_end = chi1(end);
% dchi     = chi1_end - chim(1);
% Rz = [ cos(dchi)  sin(dchi)  0;        % Z(Down)축 회전
%       -sin(dchi)  cos(dchi)  0;
%        0          0          1];
% 
% posm_rel = posm - posm(1,:);
% posm_rot = (Rz * posm_rel.').';
% velm_rot = (Rz * velm.').';
% posm_aln = posm_rot + p1_end;
% chim_aln = chim + dchi;
% 
% % 5) 시간 이어붙이기(작은 ε로 겹침 방지)
% epsT = max(1e-3, 10*(tm(2)-tm(1)));
% t2   = t1(end) + epsT + (tm - tm(1));
% 
% % 6) 합성 및 chi 미분 재계산
% time = [t1; t2];
% pos  = [pos1; posm_aln];
% vel  = [vel1; velm_rot];
% chi  = unwrap([chi1; chim_aln]);
% chid = [0; diff(chi)./diff(time)]; chid(end) = 0;
% 
% % 7) target.RefInput 재구성 (★ 이게 최종!)
% target.RefInput.pos_des     = timeseries(pos,  time);
% target.RefInput.vel_des     = timeseries(vel,  time);
% target.RefInput.chi_des     = timeseries(chi,  time);
% target.RefInput.chi_dot_des = timeseries(chid, time);
% % 필요 시: control-frame 속도도 제공
% % target.RefInput.Vel_bIc_des = timeseries(vel, time);
% 
% % 8) Variant 스위치 설정
% try
%     userStruct.variants.refInputType = RefInputEnum.TIMESERIES;
% catch
%     userStruct.variants.refInputType = 4;  % fallback
% end
% if ~exist('SimIn','var'); SimIn = struct(); end
% if ~isfield(SimIn,'Switches'); SimIn.Switches = struct(); end
% SimIn.Switches.RefTrajOn = true;
% if ~isfield(SimIn,'StopTime'); SimIn.StopTime = time(end); end
% 
% % 9) 셋업 → 시뮬 → 플롯
% simSetup;                 % (여기서 buses/variants/params 준비)
% model = 'GUAM';
% simOut = sim(model);      % 실제 시뮬 실행
% simPlots_GUAM(simOut);    % 플롯
% 
% % ---------- 로컬 함수 ----------
% function [time, pos_ned_ft, vel_ned_ftps, chi, chid] = missileSegment_to_GUAM()
%     % 네 미사일 코드(단위 m, m/s; x≈East,y≈North,z≈Up)를 실행 후
%     % NED(ft)로 변환하고 100Hz 다운샘플해서 반환
% 
%     missile.V = 250; missile.x = 1000; missile.y = -5000; missile.z = 1000;
%     target.x  = 8000; target.y = 6000;  target.z = 0;
% 
%     RTP = [target.x - missile.x, target.y - missile.y, target.z - missile.z];
%     R = norm(RTP);
%     thetaL = asin(RTP(3)/R); psiL = atan2(RTP(2), RTP(1));
%     thetaM = deg2rad(10); psiM = deg2rad(10);
% 
%     missile.xV = missile.V * cos(thetaL + thetaM) * cos(psiM + psiL);
%     missile.yV = missile.V * cos(thetaL + thetaM) * sin(psiM + psiL);
%     missile.zV = missile.V * sin(thetaL + thetaM);
% 
%     c1=5; c2=5;
%     accelY = -(missile.V)^2/R*sin(psiM) + (missile.V)^2/(4*R)*sin(2*thetaM)*tan(thetaL)*sin(2*psiM) ...
%            - c2*(missile.V)^2/R*cos(thetaM)*sin(psiM/4)*cos(psiM/4);
%     accelZ = -(missile.V)^2/R*sin(thetaM)*cos(psiM) - (missile.V)^2/R*cos(thetaM)*tan(thetaL)*(sin(psiM))^2 ...
%            - c1*(missile.V)^2/R*sin(thetaM/4)*cos(thetaM/4);
% 
%     dt=1e-4; S=100; Niter=round(S/dt); accel_max=10*9.81;
%     tt = nan(Niter,1); mx=tt; my=tt; mz=tt; mxV=tt; myV=tt; mzV=tt;
% 
%     for k=1:Niter
%         RTP = [target.x - missile.x, target.y - missile.y, target.z - missile.z];
%         R = norm(RTP);
%         m = max(-1,min(1,RTP(3)/R));
%         thetaL = asin(m);
%         thetaL = max(-pi/2+1e-6,min(pi/2-1e-6,thetaL));
%         psiL = atan2(RTP(2), RTP(1));
% 
%         thetaM_dot = accelZ/missile.V + missile.V/R * cos(thetaM)*tan(thetaL)*(sin(psiM))^2 ...
%                    + missile.V/R * sin(thetaM)*cos(psiM);
%         psiM_dot   = accelY/(missile.V*cos(thetaM)) - missile.V/R*sin(thetaM)*tan(thetaL)*sin(psiM)*cos(psiM) ...
%                    + missile.V/(R*cos(thetaM))*(sin(thetaM))^2*sin(psiM) + missile.V/R*cos(thetaM)*sin(psiM);
%         thetaM = thetaM + thetaM_dot*dt;
%         psiM   = psiM   + psiM_dot*dt;
% 
%         accelY = -(missile.V)^2/R*sin(psiM) + (missile.V)^2/(4*R)*sin(2*thetaM)*tan(thetaL)*sin(2*psiM) ...
%                - c2*(missile.V)^2/R*cos(thetaM)*sin(psiM/4)*cos(psiM/4);
%         accelZ = -(missile.V)^2/R*sin(thetaM)*cos(psiM) - (missile.V)^2/R*cos(thetaM)*tan(thetaL)*(sin(psiM))^2 ...
%                - c1*(missile.V)^2/R*sin(thetaM/4)*cos(thetaM/4);
% 
%         a_norm = hypot(accelY, accelZ);
%         if a_norm>accel_max, s=accel_max/a_norm; accelY=accelY*s; accelZ=accelZ*s; end
% 
%         missile.xV = missile.V * cos(thetaL + thetaM) * cos(psiM + psiL);
%         missile.yV = missile.V * cos(thetaL + thetaM) * sin(psiM + psiL);
%         missile.zV = missile.V * sin(thetaL + thetaM);
% 
%         missile.x = missile.x + missile.xV*dt;
%         missile.y = missile.y + missile.yV*dt;
%         missile.z = missile.z + missile.zV*dt;
% 
%         tt(k)=k*dt; mx(k)=missile.x; my(k)=missile.y; mz(k)=missile.z;
%         mxV(k)=missile.xV; myV(k)=missile.yV; mzV(k)=missile.zV;
% 
%         if R<=0.01
%             tt=tt(1:k); mx=mx(1:k); my=my(1:k); mz=mz(1:k);
%             mxV=mxV(1:k); myV=myV(1:k); mzV=mzV(1:k);
%             break;
%         end
%     end
% 
%     % 100 Hz 다운샘플
%     step=max(1,round(0.01/dt)); idx=1:step:numel(tt);
%     tt=tt(idx); mx=mx(idx); my=my(idx); mz=mz(idx); mxV=mxV(idx); myV=myV(idx); mzV=mzV(idx);
% 
%     % ENU(가정) → NED, m→ft
%     m2ft=3.280839895;
%     N=my; E=mx; D=-mz;   VN=myV; VE=mxV; VD=-mzV;
% 
%     pos_ned_ft   = [N E D]*m2ft;
%     vel_ned_ftps = [VN VE VD]*m2ft;
%     time = tt(:);
%     chi  = unwrap(atan2(VE, VN));
%     chid = [0; diff(chi)./diff(time)]; chid(end)=0;
% end

