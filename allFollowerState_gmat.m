function [follower_gmat_interp, follower_gmat_interp_v] = allFollowerState_gmat(ti,tf,dt)
% this function imports true follower position in ECI.

% IMPORT FOLLOWER DATA
fileID = fopen('ECI_state_follower.txt','r');
follower_gmat = fscanf(fileID,'%f', [7,inf]);
fclose(fileID);
follower_gmat=follower_gmat';

% SELECT DATA IN THE TIME INTERVAL OF INTEREST
simulation_time=(ti:dt:tf);

i=1;
while i<=size(follower_gmat,1)
    if follower_gmat(i,1)<ti || follower_gmat(i,1)>tf
        follower_gmat(i,:)=[];
        i=i-1;
    end
    i=i+1;
end

figure()
subplot(2,2,1)
plot3(follower_gmat(:,2),follower_gmat(:,3),follower_gmat(:,4))
xlabel('x [km]')
ylabel('y [km]')
zlabel('z [km]')
grid on
hold on
plot3(0,0,0, 'k*','markersize',3)
title('follower true trajectory in the simulation time')

subplot(2,2,2)
plot(follower_gmat(:,1)-ti,follower_gmat(:,2))
xlabel('simulation time [s]')
ylabel('x [km]')
grid on
title('true ECI x')

subplot(2,2,3)
plot(follower_gmat(:,1)-ti,follower_gmat(:,3))
xlabel('simulation time [s]')
ylabel('y [km]')
grid on
title('true ECI y')

subplot(2,2,4)
plot(follower_gmat(:,1)-ti,follower_gmat(:,4))
xlabel('simulation time [s]')
ylabel('z [km]')
grid on
title('true ECI z')

%% INTERPOLATE FOLLOWER DATA
follower_gmat_interp=zeros(length(simulation_time),4);
follower_gmat_interp(:,1)=simulation_time-ti;

follower_gmat_interp_v=zeros(length(simulation_time),4);
follower_gmat_interp_v(:,1)=simulation_time;

for i=2:1:4
    follower_gmat_interp(:,i)=spline(follower_gmat(:,1),follower_gmat(:,i), simulation_time); % cubic spline
end

for i=5:1:7
    follower_gmat_interp_v(:,i-3)=spline(follower_gmat(:,1),follower_gmat(:,i), simulation_time); % cubic spline
end


end
