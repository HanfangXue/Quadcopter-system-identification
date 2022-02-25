run('init_quadcopter_model')
run('init_quadcopter_states_2')
addpath ('models')
run('initialize_wind_dist')
run('design_pid')

%% Run PID model for reference
y_ref = 0;
out_pid = sim('closed_loop','StopTime', '50')
%%

k1 = 1/I(1); 
k2 = -g;

windamp = m;

w0_wind = 0.9802; % TODO, change this
epsilon = 0.0295; % TODO, change this

A1 = [0 0 0 0 0 0 ; 1 0 0 0 0 0 ; 0 k2 0 0 0 1; 0 0 1 0 0 0 ; 0 0 0 0 -2*w0_wind*epsilon -w0_wind^2; 0 0 0 0 1 0 ];
B1 = [k1 ; 0 ; 0 ; 0 ; 0 ; 0];
C1 = [0 1 0 0 0 0 ; 0 0 0 1 0 0];
D1 = zeros(size(C1,1),size(B1,2));
Bw = [0;0;1;0;0;0];

sys = ss(A1,B1,C1,D1);

Q = diag([0 0 0 1 0 0]);
R = 1e-1;
[K,S,E] = lqr(sys,Q,R);

G = eye(6);
H = zeros(2,6);
QN = diag([1e-5*wind_power 0 0 0 wind_power 0]);  
RN = diag([wind_power wind_power]);

[KEST,L,P] = kalman(ss(A1,[B1 G],C1,[D1 H]),QN,RN);
%% Run simulation
disp('starting sim')
out = sim('closed_loop_LQG','StopTime', '50')
disp('done sim')
%% Inspect Result
figure(1)
clf
plot(out.p);
hold on;
plot(out_pid.p, '--');
title('Position')
legend('LQG - x','LQG - y','LQG - z', 'PID - x','PID - y','PID - z')

