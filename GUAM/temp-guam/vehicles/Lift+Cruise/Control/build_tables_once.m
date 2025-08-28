function [KxTbl_lon, KiTbl_lon, KxTbl_lat, KiTbl_lat, UH, WH] = ...
  build_tables_once(Qlon_diag, Rlon_diag, XEQ_TABLE, FreeVar_Table, Trans_Table, ...
                    lpc, rho, grav, Wlon0, Qlat0, Rlat0, Wlat0)

% 그리드 크기
[~, N_trim, M_trim, L_trim] = size(XEQ_TABLE); assert(L_trim==1);

% 고정 가중 복제
Qlat = repmat(Qlat0, [1,N_trim,M_trim,L_trim]);
Rlat = repmat(Rlat0, [1,N_trim,M_trim,L_trim]);
Wlat = repmat(Wlat0, [1,N_trim,M_trim,L_trim]);

% PSO가 바꾼 lon 가중 복제
Qlon = repmat(Qlon_diag, [1,N_trim,M_trim,L_trim]);   % 6xN×M×1 (ui,wi,qi,u,w,q)
Rlon = repmat(Rlon_diag, [1,N_trim,M_trim,L_trim]);   % 11xN×M×1
Wlon = repmat(Wlon0,    [1,N_trim,M_trim,L_trim]);

% 설계 결과 담을 구조체 배열
LON = repmat(struct('Kx',[],'Ki',[], 'Ap',[],'Bp',[],'Cp',[],'Dp',[], ...
                    'C',[],'Q',[],'R',[],'XU0',[]), 1, N_trim*M_trim);
LAT = repmat(struct('Kx',[],'Ki',[], 'Ap',[],'Bp',[],'Cp',[],'Dp',[], ...
                    'C',[],'Q',[],'R',[]),           1, N_trim*M_trim);

idx = 0;
for jj = 1:M_trim % w-grid
  for ii = 1:N_trim % u-grid
    idx = idx+1;
    trim_pnt    = XEQ_TABLE(:,ii,jj,1);
    FreeVar_pnt = FreeVar_Table(:,:,jj,1);
    Trans_pnt   = Trans_Table(:,jj,1);

    % --- Longitudinal: 네가 쓰는 RSLQR 설계 함수 그대로 호출
    [lon, ~] = ctrl_lon2(lpc, trim_pnt, rho, grav, ...
                         Qlon(:,ii,jj,1), Rlon(:,ii,jj,1), Wlon(:,ii,jj,1), ...
                         FreeVar_pnt, Trans_pnt);
    LON(idx) = lon;

    % --- Lateral: 고정 설계 (한 번만 해도 되지만, 여기선 동일 인터페이스로)
    [lat, ~] = ctrl_lat2(lpc, trim_pnt, rho, grav, ...
                         Qlat(:,ii,jj,1), Rlat(:,ii,jj,1), Wlat(:,ii,jj,1), ...
                         FreeVar_pnt, Trans_pnt);
    LAT(idx) = lat;
  end
end

% === 그리드 벡터 (Prelookup 브레이크포인트)
UH = reshape(XEQ_TABLE(1,:,1,1), N_trim, 1);   % 28x1
WH = reshape(XEQ_TABLE(2,1,:,1), M_trim, 1);   % 3x1

% === K 패킹 → Prelookup 2D 보간용 (앞 2차원은 그리드)
Nu_lon= size(LON(1).Kx,1); Nx_lon = size(LON(1).Kx,2); Ni_lon = size(LON(1).Ki,2);
Nu_lat= size(LAT(1).Kx,1); Nx_lat = size(LAT(1).Kx,2); Ni_lat = size(LAT(1).Ki,2);

Kx_lon_raw = reshape([LON.Kx], Nu_lon, Nx_lon, N_trim, M_trim);  % [Nu Nx NuGrid NwGrid]
Ki_lon_raw = reshape([LON.Ki], Nu_lon, Ni_lon, N_trim, M_trim);
Kx_lat_raw = reshape([LAT.Kx], Nu_lat, Nx_lat, N_trim, M_trim);
Ki_lat_raw = reshape([LAT.Ki], Nu_lat, Ni_lat, N_trim, M_trim);

% Prelookup 규약에 맞게 permute: [NuGrid, NwGrid, Nu, Nx(or Ni)]
KxTbl_lon  = permute(Kx_lon_raw, [3 4 1 2]);  % [N_trim M_trim Nu Nx]
KiTbl_lon  = permute(Ki_lon_raw, [3 4 1 2]);  % [N_trim M_trim Nu Ni]
KxTbl_lat  = permute(Kx_lat_raw, [3 4 1 2]);
KiTbl_lat  = permute(Ki_lat_raw, [3 4 1 2]);
end
