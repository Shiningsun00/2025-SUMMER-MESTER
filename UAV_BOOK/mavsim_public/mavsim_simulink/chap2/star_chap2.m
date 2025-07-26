run('../parameters/simulation_parameters')
addpath('../chap2');
sim_time = 1000; % 시뮬레이션 수행할 총 시간 (초 단위)

set_param(mavsim_chap2, 'StopTime', num2str(sim_time));

%--- [5] 시뮬레이션 수행
sim(mavsim_chap2);