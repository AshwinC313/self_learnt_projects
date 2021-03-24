close all
clc
%% Initialization
th_int = [-pi/2 -pi/2 0];
ths = [-pi/2 pi/2 0];
x0 = [0 0 0 th_int 0 0 0 0 0 0];
Ts = [0 20];

%% Robot specifications
L1 = 1;
L2 = 1;
L3 = 1;
M1 = 1;
M2 = 1;
M3 = 1;
spec = [L1 L2 L3 M1 M2 M3];

%% PID Parameters
%PID parameters for theta 1
Kp1 = 15;
Kd1 = 7;
Ki1 = 10;
%PID parameters for theta 2
Kp2 = 15;
Kd2 = 10;
Ki2 = 10;
%PID parameters for theta 3
Kp3 = 12;
Kd3 = 8;
Ki3 = 10;
Kpid = [Kp1 Kd1 Ki1 Kp2 Kd2 Ki2 Kp3 Kd3 Ki3];

%% ODE solving
% opt1=odeset('RelTol',1e-10,'AbsTol',1e-20,NormControl','off');
[T,X] = ode45(@(t,x) r3dof(t,x,ths,spec,Kpid),Ts,x0);
%% Output
th1 = X(:,4); %theta 1 waveform
th2 = X(:,5); %theta 2 waveform
th3 = X(:,6); %theta 3 waveform
% torque inputs 
F1 = diff(X(:,10))./diff(T);
F2 = diff(X(:,11))./diff(T);
F3 = diff(X(:,12))./diff(T);
tt = 0:(T(end)/(length(F1)-1)):T(end);

x1 = L1.*sin(th1);
x2 = x1 + L2.*sin(th1+th2);
x3 = x2 + L3.*sin(th1+th2+th3);
y1 = L1.*cos(th1);
y2 = y1 + L2.*cos(th1+th2);
y3 = y2 + L3.*cos(th1+th2+th3);

%theta1 error plot
plot(T,ths(1)-th1)
grid
title('Theta-1 error')
ylabel('theta1 error (rad)')
xlabel('time (sec)')
%theta2 error plot
figure
plot(T,ths(2)-th2)
grid
title('Theta-2 error')
ylabel('theta2 error (rad)')
xlabel('time (sec)')
%theta3 error plot
figure
plot(T,ths(3)-th3)
grid
title('Theta-3 error')
ylabel('theta3 error (rad)')
xlabel('time (sec)')

%torque 1 plot
figure
plot(tt,F1)
grid
title('Torque of theta 1')
ylabel('theta1 torque')
xlabel('time (sec)')
%torque 2 plot
figure
plot(tt,F2)
grid
title('Torque of theta 2')
ylabel('theta2 torque')
xlabel('time (sec)')
%torque 3 plot
figure
plot(tt,F3)
grid
title('Torque of theta 3')
ylabel('theta3 torque')
xlabel('time (sec)')