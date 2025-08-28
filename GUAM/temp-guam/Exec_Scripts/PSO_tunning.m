function cost = PSO_tunning(qrset)

mdl = 'GUAM';

% % delete previous ISE
% try
%     evalin('base', 'clear ISE');
% catch
% end
qrset = qrset(:).';
assignin('base','ASSERT_HIT1',false);
assignin('base','ASSERT_HIT2',false);
S = Simulink.SimulationInput(mdl);
S = S.setVariable('qrset', qrset);

try
    out = sim(S); % 2023b이전 버전에서는 (S, 'ReturnWorkspaceOutputs', 'on')을 사용해야함.
catch
    cost = 1e12;
    return
end

% --- 어설션 플래그 확인 ---
hit1 = false;
hit2 = false;
try
    hit1 = evalin('base','ASSERT_HIT1');
    hit2 = evalin('base','ASSERT_HIT2');
catch
    hit1 = false;
    hit2 = false;
end
if hit1 || hit2
    cost = 1e12;   % 페널티
    return
end

% --- 정상 비용 추출 ---
ISE = [];
try
    ISE = out.get('ISE');                 % To Workspace(Array)일 때
catch
end

if isempty(ISE)
    cost = 1e12;   % 안전망
    return
end
cost = ISE(end);
end