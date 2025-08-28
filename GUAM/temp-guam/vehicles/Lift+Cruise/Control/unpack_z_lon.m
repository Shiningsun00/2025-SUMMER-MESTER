function [Qlon_diag, Rlon_diag] = unpack_z_lon(z)
v = 10.^z(:);
Qlon_diag = v(1:6);   % [ui wi qi u w q] (네 주석 순서 유지)
Rlon_diag = v(7:17);  % 11개 (스로틀8+푸셔+플랩+엘리베이터) 순서 주의
end