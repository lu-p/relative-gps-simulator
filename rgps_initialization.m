clear
close all
clc

%this script is connected with the simulink model 'rgps_model'.
% Matlab code imports data from gmat, initializes variables, checks data, launches simulink, add noise, visualizes results
% Simulink model computes receiver vectors, solve nonlinear gps system for leader and follower indipendently, subtracts solutions, transforms from ECI to LVLH

ti=0; % initial simulation time [s]
tf=ti+5600; % final simulation time [s]
dt=1; % simulation time step [s]
simulation_time=(ti:dt:tf)';

noise_int=1/10; % noise intensity of gps sensor [fraction of position estimate] how noisy is your gps sensor?

%% GPS CONSTELLATION IN ECI FROM GMAT
% GPS data do not depend on intial and final time, but only on the length of the simulation time

totsat=24; % number of satellites in the gps constellation
gps_gmat_interp=allGPSpos_gmat(ti,tf,dt); % gps_gmat_interp: first column for times, other columns for position in ECI (t, x1,y1,z1, x2, y2, z2, ..., x24, y24, z24)

%% LEADER IN ECI FROM TXT FILE

[leader_gmat_interp, leader_gmat_interp_v]=allLeaderState_gmat(ti,tf,dt); % [time, x, y, z]

%% FOLLOWER IN ECI FROM TXT FILE

[follower_gmat_interp, follower_gmat_interp_v]=allFollowerState_gmat(ti,tf,dt); % [time, x, y, z]

ECIposrel=follower_gmat_interp(:,2:4)-leader_gmat_interp(:,2:4); % relative position in ECI
ECIvelrel=follower_gmat_interp_v(:,2:4)-leader_gmat_interp_v(:,2:4); % relative velocity in ECI

figure()
subplot(2,2,1)
plot3(ECIposrel(:,1),ECIposrel(:,2),ECIposrel(:,3))
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
title('true relative position in ECI')
hold on
plot3(0,0,0,'*k') % leader position

subplot(2,2,2)
plot(simulation_time-ti, ECIposrel(:,1))
xlabel('simulation time [s]')
ylabel('x [km]')
title('true ECI x')
grid on
subplot(2,2,3)
plot(simulation_time-ti, ECIposrel(:,2))
xlabel('simulation time [s]')
ylabel('y [km]')
grid on
title('true ECI y')

subplot(2,2,4)
plot(simulation_time-ti, ECIposrel(:,3))
xlabel('simulation time [s]')
ylabel('z [km]')
grid on
title('true ECI z')

d=zeros(length(simulation_time),1);

for i=1:length(simulation_time)
    d(i)=norm(ECIposrel(i,:));
end
maxd=max(d); % max relative distance in the simulation time [km]
mind=min(d); % min relative distance in the simulation time [km]

%% FOLLOWER RELATIVE POSITION FROM ECI TO LVLH

er=zeros(length(simulation_time),3);
en=zeros(length(simulation_time),3);
et=zeros(length(simulation_time),3);

for i=1:length(simulation_time)
    er(i,:)=ECIposrel(i,:)/norm(ECIposrel(i,:)); % unit vector in r-bar
    en(i,:)=cross(ECIposrel(i,:), ECIvelrel(i,:))/norm(cross(ECIposrel(i,:), ECIvelrel(i,:))); % unit vector in h-bar
    et(i,:)=cross(er(i,:),en(i,:)); % unit vector in r-bar
end

posLVLH=zeros(length(simulation_time),3);

for i=1:length(simulation_time)
    posLVLH(i,:)=[ECIposrel(i,:)*er(i,:)', ECIposrel(i,:)*en(i,:)', ECIposrel(i,:)*et(i,:)'];
end

posLVLH2=posLVLH+leader_gmat_interp(:,2:4);

figure()
subplot(2,2,1)
plot3(posLVLH2(:,1), posLVLH2(:,2), posLVLH2(:,3))
xlabel('r-bar [km]')
ylabel('h-bar [km]')
zlabel('v-bar [km]')
title('true relative position in LVLH')
grid on
hold on
plot3(0,0,0,'k*')
subplot(2,2,2)
plot(simulation_time-ti,posLVLH(:,1))
xlabel('simulation time [s]')
ylabel('v-bar [km]')
grid on
title('true v-bar')
subplot(2,2,3)
plot(simulation_time-ti, posLVLH(:,2))
xlabel('simulation time [s]')
ylabel('h-bar [km]')
grid on
title('true h-bar')
subplot(2,2,4)
plot(simulation_time-ti, posLVLH(:,3))
xlabel('simulation time [s]')
ylabel('r-bar [km]')
grid on
title('true r-bar')

ECIvelrel_sim=[simulation_time-ti, ECIvelrel]; % for simulink

sim('rgps_model') % gps sensor

%% NOISE ADDICTION
PosEst_rel_noisy=PosEst_rel.data+rand(size(PosEst_rel.data,1),3).*PosEst_rel.data*noise_int; % uniform random noise

%% VISUALIZATION OF RESULTS

% position estimate in LVLH using GPS in the simulation time
figure() 
subplot(2,2,1)
plot3(PosEst_rel_noisy(:,1),PosEst_rel_noisy(:,2),PosEst_rel_noisy(:,3))
xlabel('v-bar [km]')
ylabel('h-bar [km]')
zlabel('r-bar [km]')
grid on
title('trajectory estimate')
hold on
plot3(0,0,0,'*k') % origin of LVLH (leader position)

subplot(2,2,2)
plot(PosEst_rel.time, PosEst_rel_noisy(:,1))
xlabel('simulation time [s]')
ylabel('v-bar [km]')
grid on
title('v-bar estimate')

subplot(2,2,3)
plot(PosEst_rel.time, PosEst_rel_noisy(:,2))
xlabel('simulation time [s]')
ylabel('h-bar [km]')
grid on
title('h-bar estimate')

subplot(2,2,4)
plot(PosEst_rel.time, PosEst_rel_noisy(:,3))
xlabel('simulation time [s]')
ylabel('r-bar [km]')
grid on
title('r-bar estimate')


ss=112; % simulink sample step

% absolute error
modPosEst=sqrt(PosEst_rel_noisy(:,1).^2+PosEst_rel_noisy(:,2).^2+PosEst_rel_noisy(:,3).^2);
modPosTrue=sqrt(posLVLH2(1:ss:end,1).^2+posLVLH2(1:112:end,2).^2+posLVLH2(1:112:end,3).^2);

abserrx=abs(PosEst_rel_noisy(:,1)-posLVLH2(1:ss:end,1));
abserry=abs(PosEst_rel_noisy(:,2)-posLVLH2(1:ss:end,2));
abserrz=abs(PosEst_rel_noisy(:,3)-posLVLH2(1:ss:end,3));

figure() 
subplot(2,2,1)
plot(PosEst_rel.time,modPosEst-modPosTrue)
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
title('abs error on module of position')

subplot(2,2,2)
plot(PosEst_rel.time,abserrx)
xlabel('relative time [s]')
ylabel('x [km]')
grid on
title('abs error on v-bar')

subplot(2,2,3)
plot(PosEst_rel.time, abserry)
xlabel('relative time [s]')
ylabel('y [km]')
grid on
title('abs error on h-bar')

subplot(2,2,4)
plot(PosEst_rel.time, abserrz)
xlabel('relative time [s]')
ylabel('z [km]')
grid on
title('abs error on r-bar')


% relative error
figure() 
subplot(2,2,1)
plot(PosEst_rel.time,(modPosEst-modPosTrue)/modPosTrue)
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
title('rel error on module of position')

subplot(2,2,2)
plot(PosEst_rel.time, abserrx/abs(posLVLH2(1:ss:end,1)))
xlabel('relative time [s]')
ylabel('x [km]')
grid on
title('rel error on v-bar')

subplot(2,2,3)
plot(PosEst_rel.time, abserry/abs(posLVLH2(1:ss:end,2)))
xlabel('relative time [s]')
ylabel('y [km]')
grid on
title('rel error on h-bar')

subplot(2,2,4)
plot(PosEst_rel.time, abserrz/abs(posLVLH2(1:ss:end,3)))
xlabel('relative time [s]')
ylabel('z [km]')
grid on
title('rel error on r-bar')















