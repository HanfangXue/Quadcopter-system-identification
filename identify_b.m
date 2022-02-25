run('init_quadcopter_model')
run('init_quadcopter_states')
run('init_noise_levels')
addpath ('models')
%%
c =1.15 %TODO change
Omega1 = sqrt(m*g/(2*k*(c^2+1)));
Omega2 = c*Omega1;
Omega = [Omega1 Omega2 Omega1 Omega2];
Omega_in.time = (0:inner_h:2)';
nbr_samples = length(Omega_in.time);
Omega_in.signals.values = zeros(nbr_samples,4);
segments =6; %TODO change to something better
segment_size = floor(nbr_samples/segments)
switch_time = [floor(segment_size/2):segment_size:nbr_samples, nbr_samples]



Omega_in.signals.values(1:switch_time(1),:) = repmat([Omega1 Omega2 Omega1 Omega2],switch_time(1),1)
for i = 2:length(switch_time)-1
    seg_size = switch_time(i) - switch_time(i-1); %needed for boundary
    if mod(i,2) == 0
        Omega_in.signals.values(switch_time(i-1)+1:switch_time(i),:) = repmat([Omega2 Omega1 Omega2 Omega1],seg_size,1)
    else
        Omega_in.signals.values(switch_time(i-1)+1:switch_time(i),:) = repmat([Omega1 Omega2 Omega1 Omega2],seg_size,1)
    end
end

%% Run simulation
disp('starting sim')
out = sim('omega_input')
disp('sim done')
%% Inspect Result
figure(1)
clf
subplot(3,1,1)
plot(out.eta)
title('Angles')
legend('\phi','\theta','\psi')
subplot(3,1,2)
plot(out.deta)
title('Angular Velocity')
legend('\phi','\theta','\psi')
subplot(3,1,3)
plot(out.p)
title('Position')
legend('x','y','z')
%% System Identification
Psidot = out.deta.data(:,3);
dat = iddata(Psidot,-2*out.Omega.data(:,1).^2 + 2*out.Omega.data(:,2).^2,inner_h )
sys = procest(dat,'p0I')  % p0 means zero poles, I means integration is enforced
figure(2)
clf
compare(dat,sys)
b_est = sys.Kp * I(3) % TODO
true_b = b