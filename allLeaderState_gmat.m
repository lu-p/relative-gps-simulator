function [leader_gmat_interp, leader_gmat_interp_v] = allLeaderState_gmat(ti,tf,dt)
% this function imports true leader position in ECI.
% this function also plot leader trajectories during the whole mission in
% the txt file (the simulation time can be a fraction of the whole mission)

% IMPORT LEADER DATA
fileID = fopen('ECI_state_leader.txt','r');
leader_gmat = fscanf(fileID,'%f', [7,inf]);
fclose(fileID);
leader_gmat=leader_gmat';

% plot positions
figure()
plot3(leader_gmat(:,2),leader_gmat(:,3),leader_gmat(:,4))
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
hold on
plot3(0,0,0,'k*')
title('leader true trajectory, whole txt file')

% SELECT DATA IN THE TIME INTERVAL OF INTEREST
simulation_time=(ti:dt:tf);

i=1;
while i<=size(leader_gmat,1)
    if leader_gmat(i,1)<ti || leader_gmat(i,1)>tf
        leader_gmat(i,:)=[];
        i=i-1;
    end
    i=i+1;
end

%% INTERPOLATE LEADER DATA
leader_gmat_interp=zeros(length(simulation_time),4);
leader_gmat_interp(:,1)=simulation_time-ti;

leader_gmat_interp_v=zeros(length(simulation_time),4);
leader_gmat_interp_v(:,1)=simulation_time;

for i=2:1:4
    leader_gmat_interp(:,i)=spline(leader_gmat(:,1),leader_gmat(:,i), simulation_time); % cubic spline
end

for i=5:1:7
    leader_gmat_interp_v(:,i-3)=spline(leader_gmat(:,1),leader_gmat(:,i),simulation_time);
end

figure()

subplot(2,2,1)
plot3(leader_gmat(:,2),leader_gmat(:,3),leader_gmat(:,4)) % pos
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
hold on
plot3(0,0,0, 'k*')
title('true leader trajectory in the simulation time')

subplot(2,2,2)
plot(leader_gmat(:,1)-ti,leader_gmat(:,2))
xlabel('simulation time [s]')
ylabel('x [km]')
grid on
title('true ECI x')

subplot(2,2,3)
plot(leader_gmat(:,1)-ti,leader_gmat(:,3))
xlabel('simulation time [s]')
ylabel('y [km]')
grid on
title('true ECI y')

subplot(2,2,4)
plot(leader_gmat(:,1)-ti,leader_gmat(:,4))
xlabel('simulation time [s]')
ylabel('z [km]')
grid on
title('true ECI z')

end
