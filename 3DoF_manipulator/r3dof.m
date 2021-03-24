function xdot = r3dof(t,x,ths,spec,Kpid)

xdot = zeros(12,1);
%% set-points
th1s = ths(1);
th2s = ths(2);
th3s = ths(3);

%% Robot Specifications
L1 = spec(1);
L2 = spec(2);
L3 = spec(3);
M1 = spec(4);
M2 = spec(5);
M3 = spec(6);
g = 9.81;

%% Inertia matrix
b11 = (M1+M2)*L1^2 +M2*L2^2 + 2*M1*L1*L2*cos(x(5))+M3*(L1^2+L2^2)+2*M3*L1*L2*cos(x(5))+M3*L3^2+M3*L1*L3*cos(x(5)+x(6))+M3*L2*L3*cos(x(6))+M3*L3*(L1*(cos(x(5)+x(6))+L2*cos(x(6))));
b12 = M2*L2^2+M2*L1*L2*cos(x(5))+M3*L1*L2*cos(x(5))+M3*L3^2+M3*L2*L3*cos(x(6))+M3*L3*(L1*cos(x(5)+x(6))+L2*cos(x(6)));
b13 = M3*L3^2+M3*L3*L1*cos(x(5)+x(6))+M3*L2*L3*cos(x(6));
b21 = M2*L2^2+M2*L1*L2*cos(x(5))+M3*L1*L2+M3*L3^2+M3*L2*L3*cos(x(6))+M3*L3*(L1*cos(x(5)+x(6))+L2*cos(x(6)));
b22 = (M2+M3)*L2^2+M3*L3^2+3*M3*L2*L3*cos(x(6));
b23 = M3*L3^2+M3*L1*L3*cos(x(5)+x(6))+M3*L2*L3*cos(x(6));
b31 = M3*L3^2+M3*L1*L3*cos(x(5)+x(6))+M3*L2*L3*cos(x(6));
b32 = M3*L3^2+M3*L2*L3*cos(x(6));
b33 = M3*L3^2;
bq = [b11 b12 b13;b21 b22 b23;b31 b32 b33];

%% C Matrix
c1 = -M2*L1*L2*sin(x(5))*(2*x(7)+x(8))-M3*L1*L2*sin(x(5))*(x(7)+x(8))-M3*L1*L3*x(7)*sin(x(5)+x(6))*(x(8)+x(9))-M3*L3*L2*sin(x(6))*(x(7)+x(8))-M3*L1*L3*(x(7)+x(9));
c2 = -(M2+M3)*L1*L2*sin(x(5))*x(7)*x(8)-M3*L3*L2*sin(x(6))*x(9)*(x(7)+x(9))-M3*L3*L2*x(8)*x(9)*sin(x(6))-M3*L1*L3*x(7)*sin(x(5)+x(6))*(x(8)+x(9))-M3*L3*L2*(x(7)+x(8))*sin(x(6))*x(9)+M2*L1*L2*x(7)*(x(7)+x(8))*sin(x(5))+M3*(x(7)+x(8))*L1*L2*x(7)*sin(x(5))-M3*L3*(x(7)+x(8)+x(9))*L1*x(7)*sin(x(5)+x(6));
c3 = -M3*L1*L3*x(7)*sin(x(5)+x(6))*(x(8)+x(9))-M3*L2*L3*(x(7)+x(8))-M3*L3*L2*(x(7)+x(8))*sin(x(6))+M3*L3*(x(7)+x(8)+x(9))*(L1*x(7)*sin(x(5)+x(6))*(x(8)+x(9))+L2*(x(7)+x(8))*sin(x(6)));
cq = [c1;c2;c3];

%% g Matrix
g1 = M1*g*L1*sin(x(4))+M2*g*(L1*sin(x(4))+L2*sin(x(5)+x(6)))+M3*g*(L1*sin(x(4))+L2*sin(x(5)+x(6))+L3*sin(x(4)+x(5)+x(6)));
g2 = M2*g*L2*sin(x(4)+x(5))+M3*g*(L2*sin(x(4)+x(5))+L3*sin(x(4)+x(5)+x(6)));
g3 = M3*g*L3*sin(x(4)+x(5)+x(6));
gq = [g1;g2;g3];

%% PID Control
% PID parameters for theta1
Kp1 = Kpid(1);
Kd1 = Kpid(2);
Ki1 = Kpid(3);
% PID parameters for theta2
Kp2 = Kpid(4);
Kd2 = Kpid(5);
Ki2 = Kpid(6);
% PID parameters for theta3
Kp3 = Kpid(7);
Kd3 = Kpid(8);
Ki3 = Kpid(9);
% de-coupled control input
f1 = Kp1*(th1s-x(4))-Kd1*x(7)+Ki1*(x(1)); 
f2 = Kp2*(th2s-x(5))-Kd2*x(8)+Ki2*(x(2));
f3 = Kp3*(th3s-x(6))-Kd3*x(9)+Ki3*(x(3));
Fhat = [f1;f2;f3];

F = bq*Fhat;

%% System states
xdot(1) = (th1s - x(4));
xdot(2) = (th2s - x(5));
xdot(3) = (th3s - x(6));
xdot(4) = x(7);
xdot(5) = x(8);
xdot(6) = x(9);
theta2dot = inv(bq)*(-cq-gq+F);
xdot(7) = theta2dot(1);
xdot(8) = theta2dot(2);
xdot(9) = theta2dot(3);
xdot(10) = F(1);
xdot(11) = F(2);
xdot(12) = F(3);


