function J = cost_ise_lon(z, model, XEQ_TABLE, FreeVar_Table, Trans_Table, ...
                          lpc, rho, grav, Wlon0, Qlat0, Rlat0, Wlat0)
try
    % 1) PSO 변수 → Qlon,Rlon 대각
    [Qlon_diag, Rlon_diag] = unpack_z_lon(z);

    % 2) 테이블 생성 (lon은 새로, lat은 고정 기본값)
    [KxTbl_lon, KiTbl_lon, KxTbl_lat, KiTbl_lat, UH, WH] = ...
        build_tables_once(Qlon_diag, Rlon_diag, XEQ_TABLE, FreeVar_Table, Trans_Table, ...
                          lpc, rho, grav, Wlon0, Qlat0, Rlat0, Wlat0);

    % 3) 시뮬에 투입 (Prelookup 보간은 모델에서 수행)
    assignin('base','KxTbl_lon',KxTbl_lon);
    assignin('base','KiTbl_lon',KiTbl_lon);
    assignin('base','KxTbl_lat',KxTbl_lat);
    assignin('base','KiTbl_lat',KiTbl_lat);
    assignin('base','UH',UH); assignin('base','WH',WH);

    simOut = sim(model, 'FastRestart','on');

    % 4) ISE 계산 (logsout에 e_u, e_w 있어야 함)
    eu = simOut.logsout.get('e_u').Values;   % u_ref - u
    ew = simOut.logsout.get('e_w').Values;   % w_ref - w
    t  = eu.Time;
    e2 = eu.Data.^2 + ew.Data.^2;
    J  = trapz(t, e2);

    if ~isfinite(J); J = 1e12; end
catch
    J = 1e12;
end
end