clc
clear all, close all
%% Measurment simulation 
C= 1; % ideal value
A=0.025;  % meas noise variance
M=1000;

SignC=C*ones(1,M); 
SignM= SignC+ A*randn(1,M);

% declaration for the monitoring purposes
inv=zeros(1,M);
K= zeros(1,M);
Xk=zeros(1,M);
Pk=zeros(1,M);

% Modelling 
phi=1;
H=1;
Qd=0.00025; 
R=A; 
Xp=0; 
Pp= 0.00025; % initial value 
I = eye(1);
% Cycles
for i=1:M
    y=SignM(i);
    inv(i)=y-H*Xp;    % innovations
    % correction step
    K(i)= Pp*H'/(H*Pp*H' + R);
    Xk(i)=Xp+K(i)*inv(i);
    Pk(i)=(I-K(i)*H)*Pp;
    Pk(i)=(Pk(i)+Pk(i)')/2;
    % time update step
    Xp=phi *Xk(i); 
    Pp = phi *Pk(i)*phi'+Qd;
end

figure;
subplot(411), plot (Xk); hold on, plot(SignM), plot (SignC), zoom on, grid on; title('xk')
subplot(412), plot (Pk); zoom on, grid on;title('Pk')
subplot(413), plot (K); zoom on, grid on;title('K')
subplot(414), plot (inv); zoom on, grid on;title('inv')
