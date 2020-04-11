clc,clear all,close all;
% Computed - Torque Control With PID design
% Written By: Rasit EVDUZEN
% 10-Mar-2020

%% Algorithm Parameter
% Controller & Integration Parameters
tsimulation = 4;       % Simulation time
Ts = 0.001;              % Sampling time
t = 0:Ts:tsimulation;   % Time Vector
Trq = zeros(2,1);        % Each Joint Torque initial value
ksi = 1 ;               % Damped Ratio for controller
wn = 1 / (Ts);          % Natural Frequency for controller
Kp = 20;                 % P gain for controller Ts = 0.01 Kp = 10 Kv = 5
Kv = 5;               % D gain for controller
Ki = 10;                % I gain for controller
% Kp = wn^2;              % P gain for controller
% Kv = 2*ksi*wn;          % D gain for controller

% Manipulator Parameters
L1 = 1;                 % Robot link1
L2 = 1;                 % Robot link2
m1 = 1;                 % link1 mass
m2 = 1;                 % link2 mass
g = 9.81;               % gravitational force


%% Each Link Desired Trajectory
% Trajectory - 1
% p   = [1 1];      % Robot set position and Path generation      
% [teta1,teta2] = ik(p(1,1),p(1,2),L1,L2);
% [P1,P1d,P1dd] = pth(0,teta1,0,0,tsimulation,Ts);
% [P2,P2d,P2dd] = pth(0,teta2,0,0,tsimulation,Ts);

% Trajectory - 2
% P1   = (t.^3)/6;
% P1d  = (t.^2)/2;
% P1dd = t;
% 
% P2   = (t.^3)/6;
% P2d  = (t.^2)/2;
% P2dd = t;

% Trajectory - 3
gd = 0.1;
P1   = gd*sin(pi*t);
P1d  = gd*pi*cos(pi*t);
P1dd = -gd*pi^2*sin(pi*t);

P2   = gd*cos(pi*t);
P2d  = -gd*pi*sin(pi*t);
P2dd = -gd*pi^2*cos(pi*t);



q = [P1;P2;P1d;P2d;P1dd;P2dd];
qini = q(1:4,1);
qini(5,1) = P1(1,1) - qini(1);
qini(6,1) = P2(1,1) - qini(2);
xhat = L1*cos(deg2rad(P1)) + L2*cos(deg2rad(P1)+deg2rad(P2));  % Calculate Forward Kinematics
yhat = L1*sin(deg2rad(P1)) + L2*sin(deg2rad(P1)+deg2rad(P2));  % Calculate Forward Kinematics

cplot = @(r,x0,y0) plot(x0 + r*cos(linspace(0,2*pi)),y0 + r*sin(linspace(0,2*pi)),'k--');
figure(1)
%% Algorithm
for j=1:length(P1)
    clf 
    


    % Calculate RK
    [qhat,M,N] = RK4(m1,m2,L1,L2,g,qini,Trq(:,j),Ts);
    qini = qhat;
    q1hat(1,j) = qhat(1);   % Calculate link-1 angular position by RK4
    q2hat(1,j) = qhat(2);   % Calculate link-2 angular position by RK4
    q3hat(1,j) = qhat(3);   % Calculate link-1 angular velocity by RK4
    q4hat(1,j) = qhat(4);   % Calculate link-2 angular velocity by RK4
    
    % Calculate error and Torque with computed torque pd control
    e1(j)   = P1(1,j)  - q1hat(1,j);
    e2(j)   = P2(1,j)  - q2hat(1,j);
    qini(5) = e1(j);
    qini(6) = e2(j);
    ed1(j)  = P1d(1,j) - q3hat(1,j);
    ed2(j)  = P2d(1,j) - q4hat(1,j);
    tmp1 = P1dd(1,j) + Kv*ed1(j) + Kp*e1(j) + Ki*qini(5);
    tmp2 = P2dd(1,j) + Kv*ed2(j) + Kp*e2(j) + Ki*qini(6);   
    t1 = M(1,1)*tmp1 + M(1,2)*tmp2 + N(1);
    t2 = M(1,2)*tmp1 + M(2,2)*tmp2 + N(2);
    Trq(:,j+1) = [t1;t2];
    
    % Plot Data

%     subplot(2,2,[1,4])
%     set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
%     A1x = [0 L1*cos(deg2rad(P1(j)))];
%     A1y = [0 L1*sin(deg2rad(P1(j)))];
%     A2x = [A1x(:,2) A1x(:,2)+L2*cos(deg2rad(P1(j)+P2(j)))];
%     A2y = [A1y(:,2) A1y(:,2)+L2*sin(deg2rad(P1(j)+P2(j)))];
%     plot(A1x,A1y,'r','LineWidth',2)
%     hold on,grid on
%     cplot(L1+L2,0,0)
%     plot(xhat,yhat,'m')
%     scatter(xhat(1,1),yhat(1,1),'r','filled')
%     scatter(xhat(1,end),yhat(1,end),'r','filled')
%     plot(A2x,A2y,'b','LineWidth',2)
%     axis equal
%     axis([-(L1+L2)-1 (L1+L2)+1 -(L1+L2)-1 (L1+L2)+1])
%     xlabel('X - Axis'),ylabel('Y - Axis'),title('2 Dof Robot Arm Computed Torque Control with PID')
%     drawnow
end

%%
figure(2)
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);

annotation('textbox', [0.015, 0.83, 0.1, 0.1], 'String', "ALGORITHM PARAMTERS" )
annotation('textbox', [0.015, 0.8, 0.1, 0.1], 'String', "Damping Ratio (\xi) = " + ksi)
annotation('textbox', [0.015, 0.77, 0.1, 0.1], 'String', "Natural Frequency (\omega) = " + wn)
annotation('textbox', [0.015, 0.74, 0.1, 0.1], 'String', "Kp = " + Kp)
annotation('textbox', [0.015, 0.71, 0.1, 0.1], 'String', "Kv = " + Kv)
annotation('textbox', [0.015, 0.68, 0.1, 0.1], 'String', "Ki = " + Ki)
annotation('textbox', [0.015, 0.65, 0.1, 0.1], 'String', "Simulation Time = " + tsimulation)
annotation('textbox', [0.015, 0.62, 0.1, 0.1], 'String', "Sampling Time = " + Ts)

subplot(521)
plot(t,P1,'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Position Profile')
plot(t,q1hat,'r--','linewidth',2)
legend({'Desired Position','Actual Position'})

subplot(522)
plot(t,(P1-q1hat),'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Position Error')
legend({'Joint-1 Position Error'})

subplot(523)
plot(t,P2,'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Position Profile')
plot(t,q2hat,'r--','linewidth',2)
legend({'Desired Position','Actual Position'})

subplot(524)
plot(t,(P2-q2hat),'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Position Error')
legend({'Joint-2 Position Error'})

subplot(525)
plot(t,P1d,'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Velocity Profile')
plot(t,q3hat,'r--','linewidth',2)
legend({'Desired Position','Actual Velocity'})


subplot(526)
plot(t,(P1d-q3hat),'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Velocity Error')
legend({'Joint-2 Velocity Error'})

subplot(527)
plot(t,P2d,'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Velocity Profile')
plot(t,q4hat,'r--','linewidth',2)
legend({'Desired Velocity','Actual Velocity'})

subplot(528)
plot(t,(P2d-q4hat),'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Velocity Error')
legend({'Joint-2 Velocity Error'})

subplot(529)
plot(Trq(1,:),'r','linewidth',2),grid minor,hold on,title('Joint-1 Torque')
legend({'Joint-1 Torque Value'})

subplot(5,2,10)
plot(Trq(2,:),'r','linewidth',2),grid minor,hold on,title('Joint-2 Torque')
legend({'Joint-2 Torque Value'})
drawnow
