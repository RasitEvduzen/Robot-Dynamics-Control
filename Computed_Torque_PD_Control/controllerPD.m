%% COMPUTED-TORQUE PD CONTROLLER

%% Trajectory parameters
g1 = 0.1; g2 = 0.1;

%% Desired Trajectories
qd(1) = g1*sin(pi*timeStep);
qd(2) = g2*cos(pi*timeStep);
qdp(1) = g1*pi*cos(pi*timeStep);
qdp(2) = -g2*pi*sin(pi*timeStep);
qdpp(1) = -g1*pi^2*sin(pi*timeStep);
qdpp(2) = -g2*pi^2*cos(pi*timeStep);

%% Tracking Error and Derivative
e(1) = qd(1)-q(1);
e(2) = qd(2)-q(2);
ep(1) = qdp(1)-q(3);
ep(2) = qdp(2)-q(4);



%% M(q) Matrix
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(q(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;

IM=inv(M);

%% N(q,qd) Matrix
N(1,1) = -m2*L1*L2*(2*q(3)*q(4)+q(4)^2)*sin(q(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(q(1))+m2*g*L2*cos(q(1)+q(2));
N(2,1) = m2*L1*L2*q(3)^2*sin(q(2))+m2*g*L2*cos(q(1)+q(2));

%% Control Torques
u1 = -kv*ep(1)-kp*e(1);
u2 = -kv*ep(2)-kp*e(2);


s1 = qdpp(1) - u1;
s2 = qdpp(2) - u2;
t1 = M(1,1)*s1+M(1,2)*s2+N(1,1);
t2 = M(1,2)*s1+M(2,2)*s2+N(2,1);
