clc,clear all,close all;
%% Manipulator parameters
L1 = 1;  L2 = 1;
m1 = 1;  m2 = 1;
g = 9.81;

%% Controller parameters
stateDim = 4;
tr = 5;
ts = 0.01;
totalStep = round( tr/ts );

ksi = 1 ;               % Damped Ratio for controller
wn = 1 / (ts);          % Natural Frequency for controller
% kp = wn^2; kv = 2*wn; 
kp = 100; kv = 20;

timeStep = 0.0;
it = 1;
ip = 1;

t(ip) = timeStep;

q(1:stateDim)  = zeros(1,stateDim); % q = [theta1 theta2 theta1dot theta2dot]' 
qp(1:stateDim) = zeros(1,stateDim); % qp = qdot

for k = 1:totalStep

    
   controllerPD 
   rungeKutta4th
   
   timeStep = timeStep + ts;
   ip = ip + 1;
   t(ip) = timeStep;

   error1(ip) = e(1);
   error2(ip) = e(2);

   torque1(ip) = t1;
   torque2(ip) = t2;

   q1(ip) = q(1);
   q2(ip) = q(2);
   q1d(ip) = q(3);
   q2d(ip) = q(4);
  
end


%% Plot Data

set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
tr1 = g1*sin(pi*t);
tr2 = g2*cos(pi*t);
tr1d = g1*pi*cos(pi*t);
tr2d = -g2*pi*sin(pi*t);

annotation('textbox', [0.015, 0.83, 0.1, 0.1], 'String', "ALGORITHM PARAMTERS" )
annotation('textbox', [0.015, 0.8, 0.1, 0.1], 'String', "Damping Ratio (\xi) = " + ksi)
annotation('textbox', [0.015, 0.77, 0.1, 0.1], 'String', "Natural Frequency (\omega) = " + wn)
annotation('textbox', [0.015, 0.74, 0.1, 0.1], 'String', "Kp = " + kp)
annotation('textbox', [0.015, 0.71, 0.1, 0.1], 'String', "Kv = " + kv)
annotation('textbox', [0.015, 0.68, 0.1, 0.1], 'String', "Simulation Time = " + tr)
annotation('textbox', [0.015, 0.65, 0.1, 0.1], 'String', "Sampling Time = " + ts)

subplot(421)
plot(t,tr1,'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Position Profile')
plot(t,q1,'r--','linewidth',2)
legend({'Desired Position','Actual Position'})


subplot(422)
plot(t,(tr1-q1),'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Position Error')
legend({'Joint-1 Position Error'})

subplot(423)
plot(t,tr2,'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Position Profile')
plot(t,q2,'r--','linewidth',2)
legend({'Desired Position','Actual Position'})

subplot(424)
plot(t,(tr2-q2),'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Position Error')
legend({'Joint-2 Position Error'})

subplot(425)
plot(t,tr1d,'k','linewidth',2),grid minor,hold on,title('Joint-1 Path Velocity Profile')
plot(t,q1d,'r--','linewidth',2)
legend({'Desired Position','Actual Velocity'})

subplot(426)
plot(t,torque1,'r','linewidth',2),grid minor,hold on,title('Joint-1 Torque')
legend({'Joint-1 Torque Value'})

subplot(427)
plot(t,tr2d,'k','linewidth',2),grid minor,hold on,title('Joint-2 Path Velocity Profile')
plot(t,q2d,'r--','linewidth',2)
legend({'Desired Velocity','Actual Velocity'})

subplot(428)
plot(t,torque2,'r','linewidth',2),grid minor,hold on,title('Joint-2 Torque')
legend({'Joint-2 Torque Value'})



