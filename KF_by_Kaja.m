clc, clear all;

% state(1) X_k= F_(k-1)+G_(k-1)*u_(k-1)+W_(k-1)
% output(2) y_k= H_k*x_k*v_k

%Predpoved vektoru X_k daný skalárom k-1
    % X_k/(k-1)= F_(k-1)*X_(k-1)+G_(k-1)*u_(k-1)
%Chyba kovariancnej matice
    % P_k/(k-1)= F_(k-1)*F_(k-1)+Q_(k-1)
%Kalman Gain=K_k
    % K_k= P_k/(k-1)H'_k*(H_k*P_k*H'+R_k)^(-1)
%Info o merani a vypoc. skalare (state estim update)
    %X_k= x_k/(k-1)+K_k(Z_k-H_k*X_k/(k-1))
%Error covariance update
    %P_k=(1-K_k*H_k)*P_k/(k-1)

%% Simulacia a meranie
clear all
N=1000;
dt=0.001;
t=dt*(1:N);
F=[1 dt
   0 1]; %matica mernaia
G=[-1/2*dt^2
    -dt]; %process noise couplig matrix
H=[1 0]; %measurement matrix
Q=[0 0
    0 0]; %bez sumu, process noise covariance matrix
u=9.80665;
I= eye(2);
y0=100;
v0=0; %vychodiskova pozicia
xt=zeros(2,N);
xt(:,1)=[y0;v0]

%% vychodiskove stavy z odhadu
for k=2:N
xt(:,k)=F*xt(:,k-1)+G*u;
end

R=4;
v=sqrt(R)*randn(1,N);
z=H*xt+v;


%% Kalman filter
x=zeros(2,N)
x(:,1)=[10 0]
P=[50 0
    0 0.01];
for k=2:N
x(:,k)=F*x(:,k-1)+G*u; %skalar vector
P=F*P*F'+Q; %kovariancna matica
K=P*H'/(H*P*H'+R); %Kalman Gain 
x(:,k)=x(:,k)+K*(z(k)-H*x(:,k)); %update a korekcia
P=(I-K*H)*P; %update kovariancnej matice 
end

figure(1);
subplot(211);
plot(t,z,'g-',t,x(1,:),'b--','Linewidth',2);
hold on;
plot (t,xt(1,:),'r:','Linewidth',1.5);
legend('Merane','Odhadovane','Prave');
subplot(212);
plot(t,x(2,:),'Linewidth',2);
hold on;
plot(t,xt(2,:),'r:','Linewidth',1.5);
legend('Odhadovane', 'Prave');

figure(2);
subplot(211);
plot(t,x(1,:)-xt(1,:),'b--','Linewidth',2);

legend('Merane','Odhadovane','Prave');
subplot(212);
plot(t,x(2,:)-xt(2,:),'Linewidth',2);

legend('Odhadovane','Prave');




    

