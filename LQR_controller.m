clear all;
close all;
clc;
variables;


A = [0 1 ; (C2*C3-C1*C4)/C2 0]
B = [0 ; -R*(C4/C2)]  
D= [0 ; 0];
C =[1 0 ; 0 1];

Q = diag([1000 ; 1]); 

R=1% not want a control voltage that is too big,
                                % prevent wastage of energy.
sys_c = ss(A,B,C,D)
%%%%%%%%discrete lqr %%%%%%%%%%%
Ts=0.004
sys_d=c2d(sys_c,Ts,'zoh')
a     = sys_d.A;
b     = sys_d.B;
c     = sys_d.C;
dd    = sys_d.D;
disp('LQR coefficients');
%[K,S,e] = lqr(A,B,Q,R)
K = dlqr(a,b,Q,R)


t_final= 10;
x0 =[0,6];

SIM=sim('SIM_lqr_d.slx')

t=SIM.X.time;
x1=SIM.X.signals.values(:,1);
x2=SIM.X.signals.values(:,2);
u1=SIM.U.signals.values(:,1);
%S=SIM.S.signals.values(:,1);

figure

subplot(3,1,1)
plot(t,x1,'LineWidth',3)
ylabel('angle')
grid on
legend('theta(Body)')

subplot(3,1,2)
plot(t,x2,'LineWidth',3)
ylabel('angularVelocity')
grid on
legend('thetadot(Body)')

subplot(3,1,3)
plot(t,u1(1:end-2),'LineWidth',3)
grid on
legend('Control Signal')
xlabel('Time')

figure
step(sys_d)
figure
impulse(sys_d)
t=0:0.01:10;
unitstep = t>=0;
u = 70*unitstep;

%lsim(sys_clo,u,t)
% 
% Cn = [0 1];
% sys_ss = ss(A,B,Cn,0);
% Nbar = rscale(sys_ss,K)
% 
% sys_clo=ss(A-B*K,B*Nbar,C,D)

%[A,B,c_,d_] = tf2ss(num,den)
%kk=lqr(aa,bb,Q,R)