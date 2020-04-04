% FOURTH-ORDER RUNGE-KUTTA INTEGRATION

% M(q) Matrix
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(q(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;

invM = inv(M);

%N(q,qd) Matrix
N(1,1) = -m2*L1*L2*(2*q(3)*q(4)+q(4)^2)*sin(q(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(q(1))+m2*g*L2*cos(q(1)+q(2));
N(2,1) = m2*L1*L2*q(3)^2*sin(q(2))+m2*g*L2*cos(q(1)+q(2));


% State Equations
qp(1) = q(3);
qp(2) = q(4);
qp(3) = invM(1,1)*(-N(1,1)+t1)+invM(1,2)*(-N(2,1)+t2);
qp(4) = invM(1,2)*(-N(1,1)+t1)+invM(2,2)*(-N(2,1)+t2);

for i = 1:stateDim
    tempx(i) = q(i);
    tempxp(i) = qp(i);
end

for i = 1:stateDim
    q(i) = q(i)+.5*ts*qp(i);
end
timeStep = timeStep+.5*ts;

% M(q) Matrix
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(q(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;

invM = inv(M);

%N(q,qd) Matrix
N(1,1) = -m2*L1*L2*(2*q(3)*q(4)+q(4)^2)*sin(q(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(q(1))+m2*g*L2*cos(q(1)+q(2));
N(2,1) = m2*L1*L2*q(3)^2*sin(q(2))+m2*g*L2*cos(q(1)+q(2));


% State Equations
qp(1) = q(3);
qp(2) = q(4);
qp(3) = invM(1,1)*(-N(1,1)+t1)+invM(1,2)*(-N(2,1)+t2);
qp(4) = invM(1,2)*(-N(1,1)+t1)+invM(2,2)*(-N(2,1)+t2);

for i = 1:stateDim
    tempxp(i) = tempxp(i)+2*qp(i);
    q(i) = tempx(i)+.5*ts*qp(i);
end

% M(q) Matrix
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(q(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;

invM = inv(M);

%N(q,qd) Matrix
N(1,1) = -m2*L1*L2*(2*q(3)*q(4)+q(4)^2)*sin(q(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(q(1))+m2*g*L2*cos(q(1)+q(2));
N(2,1) = m2*L1*L2*q(3)^2*sin(q(2))+m2*g*L2*cos(q(1)+q(2));


% State Equations
qp(1) = q(3);
qp(2) = q(4);
qp(3) = invM(1,1)*(-N(1,1)+t1)+invM(1,2)*(-N(2,1)+t2);
qp(4) = invM(1,2)*(-N(1,1)+t1)+invM(2,2)*(-N(2,1)+t2);

for i = 1:stateDim
    tempxp(i) = tempxp(i)+2*qp(i);
    q(i) = tempx(i)+ts*qp(i);
end

timeStep = timeStep+.5*ts;

% M(q) Matrix
M(1,1) = (m1+m2)*L1^2+m2*L2^2+2*m2*L1*L2*cos(q(2));
M(1,2) = m2*L2^2+m2*L1*L2*cos(q(2));
M(2,1) = M(1,2);
M(2,2) = m2*L2^2;

invM = inv(M);

%N(q,qd) Matrix
N(1,1) = -m2*L1*L2*(2*q(3)*q(4)+q(4)^2)*sin(q(2));
N(1,1) = N(1,1)+(m1+m2)*g*L1*cos(q(1))+m2*g*L2*cos(q(1)+q(2));
N(2,1) = m2*L1*L2*q(3)^2*sin(q(2))+m2*g*L2*cos(q(1)+q(2));


% State Equations
qp(1) = q(3);
qp(2) = q(4);
qp(3) = invM(1,1)*(-N(1,1)+t1)+invM(1,2)*(-N(2,1)+t2);
qp(4) = invM(1,2)*(-N(1,1)+t1)+invM(2,2)*(-N(2,1)+t2);

for i = 1:stateDim
    q(i) = tempx(i)+ts*(tempxp(i)+qp(i))/6;
end

