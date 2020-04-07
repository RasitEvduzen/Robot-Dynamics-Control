function [x,M,N] = RK4(m1,m2,L1,L2,g,x,Trq,Ts)
% 2 Dof Robotic Arm Nonlinear Dynamic Model
% q     = input vector   [] q1 = theta 1, q2 = theta 2, q3 = W1, q4 = W2
% qhat  = output vector  [] qhat1 = q3 = W1, qhat2 = q4 = W2
% qhat3 = alpha1, qhat4 = alpha2,
% Theta = angular displacement
% W     = angular velocity
% Alpha = angular acceleration
% t     = Torque  input

x0 = x;           % state initial value
% -------System Matrix M&N--------------%
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(x(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(x(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;
invM = inv(M);

N(1,1) = -m2*L1*L2*(2*x(3)*x(4)+x(4)^2)*sin(x(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(x(1))+m2*g*L2*cos(x(1)+x(2));
N(2,1) = m2*L1*L2*x(3)^2*sin(x(2))+m2*g*L2*cos(x(1)+x(2));


% Nonlinear State Space System
F = [x(3);
     x(4);
     invM(1,1)*(-N(1,1)+Trq(1,1))+invM(1,2)*(-N(2,1)+Trq(2,1));
     invM(1,2)*(-N(1,1)+Trq(1,1))+invM(2,2)*(-N(2,1)+Trq(2,1))];
% ------------------------------
for i=1:length(x)
    K(i,1) = Ts*F(i);
end
% -----------------------------
x = x0 + K(:,1)/2;
% -------System Matrix M&N--------------%
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(x(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(x(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;
invM = inv(M);

N(1,1) = -m2*L1*L2*(2*x(3)*x(4)+x(4)^2)*sin(x(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(x(1))+m2*g*L2*cos(x(1)+x(2));
N(2,1) = m2*L1*L2*x(3)^2*sin(x(2))+m2*g*L2*cos(x(1)+x(2));


% Nonlinear State Space System
F = [x(3);
     x(4);
     invM(1,1)*(-N(1,1)+Trq(1,1))+invM(1,2)*(-N(2,1)+Trq(2,1));
     invM(1,2)*(-N(1,1)+Trq(1,1))+invM(2,2)*(-N(2,1)+Trq(2,1))];
% ------------------------------

for i=1:length(x)
    K(i,2) = Ts*F(i);
end
% -----------------------------
x = x0 + K(:,2)/2;
% -------System Matrix M&N--------------%
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(x(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(x(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;
invM = inv(M);

N(1,1) = -m2*L1*L2*(2*x(3)*x(4)+x(4)^2)*sin(x(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(x(1))+m2*g*L2*cos(x(1)+x(2));
N(2,1) = m2*L1*L2*x(3)^2*sin(x(2))+m2*g*L2*cos(x(1)+x(2));


% Nonlinear State Space System
F = [x(3);
     x(4);
     invM(1,1)*(-N(1,1)+Trq(1,1))+invM(1,2)*(-N(2,1)+Trq(2,1));
     invM(1,2)*(-N(1,1)+Trq(1,1))+invM(2,2)*(-N(2,1)+Trq(2,1))];
% ------------------------------

for i=1:length(x)
    K(i,3) = Ts*F(i);
end
% -----------------------------
x = x0 + K(:,3);
% -------System Matrix M&N--------------%
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(x(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(x(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;
invM = inv(M);

N(1,1) = -m2*L1*L2*(2*x(3)*x(4)+x(4)^2)*sin(x(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(x(1))+m2*g*L2*cos(x(1)+x(2));
N(2,1) = m2*L1*L2*x(3)^2*sin(x(2))+m2*g*L2*cos(x(1)+x(2));


% Nonlinear State Space System
F = [x(3);
     x(4);
     invM(1,1)*(-N(1,1)+Trq(1,1))+invM(1,2)*(-N(2,1)+Trq(2,1));
     invM(1,2)*(-N(1,1)+Trq(1,1))+invM(2,2)*(-N(2,1)+Trq(2,1))];
% ------------------------------

for i=1:length(x)
    K(i,4) = Ts*F(i);
end
% -----------------------------
for i=1:length(x)
   x(i) = x0(i) + K(i,1)/6 + K(i,2)/3 + K(i,3)/3 + K(i,4)/6;
end
% -------System Matrix M&N--------------%
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(x(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(x(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;
invM = inv(M);

N(1,1) = -m2*L1*L2*(2*x(3)*x(4)+x(4)^2)*sin(x(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(x(1))+m2*g*L2*cos(x(1)+x(2));
N(2,1) = m2*L1*L2*x(3)^2*sin(x(2))+m2*g*L2*cos(x(1)+x(2));

% ------------------------------
end

