clear; clc;

global POLY % Define global variable to switch between aero/propulsive L+C databases
POLY = 1; % 1=> use polynomial aero/propulsive (A/P) database, 0=> use strip theory s-function A/P database

% Specify necessary constants (e.g., gravity and air density)
rho  = 0.0023769; % slugs/ft^3
grav = 32.17405; % ft/sec^2
ft2kts = 1/(1852.0/0.3048/3600); % Obtained from setUnits... Old = 0.592484; % Conversion from ft/sec to kts

% *************************************************************************
fprintf('Control Scheduler \n');

% *********************** USER INPUT SECTION ******************************
fprintf('Concatenating Trim Files\n');
% Provide the input trim path and filenames (Trim_Ver1p0)
% fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver1p0';
% trim_fnames = {'Trim_Case_-6.1_RInf_WH-11.7_XEQ.mat', 'Trim_Case_6_RInf_WH0_XEQ.mat', 'Trim_Case_6.1_RInf_WH11.7_XEQ.mat'}; % Trim_Ver1p0
% out_trim_fname  = 'Trim_poly_XEQ_Concat.mat';% Trim_Ver1p0
% out_cntr_fname = './trim_table_Poly_Concat.mat'; % Output file for Trim_Ver1p0 (need to change file read in SetupControl.m)

% % Provide the input trim path and filenames (Trim_Ver2p0)
% fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver2p0';
% trim_fnames = {'Trim_Case_-6.2_RInf_WH-11.7_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat', 'Trim_Case_6.2_RInf_WH11.7_XEQ.mat'}; % Trim_Ver2p0
% out_trim_fname  = 'Trim_poly_XEQ_ConcatV2p0.mat'; % Trim_Ver2p0
% out_cntr_fname = './trim_table_Poly_ConcatVer2p0.mat'; % Output file for Trim_Ver2p0 (need to change file read in SetupControl.m)

% Provide the input trim path and filenames (Trim_Ver2p0)
%fpath       = 'C:\Users\macheson\Desktop\TTT_AS_Git\GTM-GUAM-simulation\vehicles\Lift+Cruise\Trim\Trim_Figs\Trim_Ver3p0';
fpath       = 'C:\Users\mjki1\Desktop\2025-SUMMER-MESTER\GUAM\temp-guam\vehicles\Lift+Cruise\Trim';

%trim_fnames = {'Trim_Case_-6.3_RInf_WH-11.7_XEQ.mat', 'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
%trim_fnames = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat', 'Trim_Case_6.2_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
% trim_fnames  = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat','Trim_Case_6.3_RInf_WH0_XEQ.mat','Trim_Case_6.3_RInf_WH11.7_XEQ.mat'}; % Trim_Ver3p0
% trim_fnames  = {'Trim_Case_-6.3_RInf_WH-7.5_XEQ.mat','Trim_Case_6.3_RInf_WH0_XEQ.mat'}; % Trim_Ver3p0
 trim_fnames  = {'Trim_Case_-6.5_RInf_WH-7.5_XEQ.mat','Trim_Case_6.4_RInf_WH0_XEQ.mat','Trim_Case_6.4_RInf_WH11.7_XEQ.mat'}; % Trim_Ver3p0
% out_trim_fname  = 'Trim_poly_XEQ_ConcatV3p0.mat'; % Trim_Ver3p0
% out_cntr_fname = './trim_table_Poly_ConcatVer3p0.mat'; % Output file for Trim_Ver3p0 (need to change file read in SetupControl.m)
out_trim_fname  = 'Trim_poly_XEQ_ConcatV4p0.mat'; % Trim_Ver4p0
out_cntr_fname = './trim_table_Poly_ConcatVer4p0.mat'; % Output file for Trim_Ver4p0 (need to change file read in SetupControl.m)
% *********************** END OF USER INPUT SECTION ***********************

% Specify the filename for the trim output file that contains the combined files
out_trim_fpath       = 'C:\Users\mjki1\Desktop\2025-SUMMER-MESTER\GUAM\temp-guam\vehicles\Lift+Cruise';
out_cntr_fpath       = 'C:\Users\mjki1\Desktop\2025-SUMMER-MESTER\GUAM\temp-guam\vehicles\Lift+Cruise';
%Concatenate_Trim_Files; % Concatenate trim files 

% Process the data 
load(fullfile(out_trim_fpath, out_trim_fname)); % Load resultant trim table for Trim version

% Parse the trim table
XEQ_TABLE = XEQ;
if any(any(isnan(XEQ_TABLE)))
    keyboard
end

% Determine size of gain scheduling arrays
%    UH     WH      R 
[~, N_trim, M_trim, L_trim] = size(XEQ_TABLE); 

% Initial output table
XEQ0 = zeros(21,N_trim,M_trim,L_trim);

% Build the aircraft (aero/propulsive modeling..)
if POLY  
  SimIn.numEngines = 9;
  lpc = LpC_model_parameters(SimIn);
else
  lpc = build_Lift_plus_Cruise();
end

model = 'GUAM';               % 네 시뮬링크 모델명

% --- 고정(기본) 가중들: lat은 고정, lon은 PSO가 덮어씀
%  * 네 ctrl_scheduler_GUAM에 있던 초기값을 그대로 복사/수정
Wlon0 = [ 1 1 1 1 1 1 1 1 1 1000 10000000 0.1]'; % (omp1..9, dele, dflap, th) 순서였던 거 주의
Qlat0 = [1 1 0 0 0 0]';
Rlat0 = [1 1 1 1 1 1 1 1 1 1]';
Wlat0 = [1 1 1 1 1 1 1 1 1000 1000 1]';  % LAT도 필요시 고정

% --- PSO 변수 경계 (lon용: Qlon diag 6개 + Rlon diag 11개 = 17개)
%     Qlon diag: [ui wi qi u w q], Rlon diag: 11채널
lb = log10([ 1e-6*ones(1,6),   1e-4*ones(1,11) ]);
ub = log10([ 1e+6*ones(1,6),   1e+4*ones(1,11) ]);
nvar = numel(lb);

opts = optimoptions('particleswarm', ...
  'SwarmSize', 24, 'MaxIterations', 40, ...
  'Display','iter', 'UseParallel', false, ... % parpool 있으면 true
  'FunctionTolerance', 1e-3);

% --- 목적함수: ISE(e_u,e_w) 최소화
obj = @(z) cost_ise_lon(z, model, XEQ_TABLE, FreeVar_Table, Trans_Table, ...
                        lpc, rho, grav, Wlon0, Qlat0, Rlat0, Wlat0);

% --- 실행
[z_opt, Jbest] = particleswarm(obj, nvar, lb, ub, opts);

% --- 최적 Q,R 복원 & 최종 테이블 생성/워크스페이스 반영 (검증/운영용)
[Qlon_diag, Rlon_diag] = unpack_z_lon(z_opt);
[KxTbl_lon, KiTbl_lon, KxTbl_lat, KiTbl_lat, UH, WH] = ...
    build_tables_once(Qlon_diag, Rlon_diag, XEQ_TABLE, FreeVar_Table, Trans_Table, ...
                      lpc, rho, grav, Wlon0, Qlat0, Rlat0, Wlat0);

assignin('base','KxTbl_lon',KxTbl_lon);
assignin('base','KiTbl_lon',KiTbl_lon);
assignin('base','KxTbl_lat',KxTbl_lat);
assignin('base','KiTbl_lat',KiTbl_lat);
assignin('base','UH',UH); assignin('base','WH',WH);

fprintf('\nBest ISE = %.3g\n', Jbest);
disp('Qlon diag:'), disp(Qlon_diag(:).');
disp('Rlon diag:'), disp(Rlon_diag(:).');
