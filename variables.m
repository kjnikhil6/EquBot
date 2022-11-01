clc;
clear all;
%close all;
g = 9.8067;     %Acceleration due to gravity(m/s^2)


%=================MAIN_BODY=================


arduinoBoard= 0.0250;      % Mass of the board(Kg)
stator = 0.180
a4988 = 0.0015
mpu6050 = 0.0021
longBreadboard = 0.75
acylicSide = 0.0375
acrylicFlat = 0.080

%Mounting_Brackets
mounting_brackets_nuts_bolts = 0.020
mounting_brackets = 0.090
totMountBrac=2*mounting_brackets_nuts_bolts+2*mounting_brackets



Mb = arduinoBoard+mpu6050+(2*a4988)+(2*acrylicFlat+2*acylicSide)...
     +longBreadboard...
     +totMountBrac+(2*stator) % Mass of the body and 2 motor stators.(Kg)             

Lb = 0.0285                    % Distance of center of mass to wheel axle.(m)
Jb = 0.85*Mb*Lb*Lb            % Moment inertia of Main Body reference to the center of mass.(Kgm2)




%===========STEPPER_MOTORS=========

stepper_motor = .270;          %Mass od Nema 17 stepper motor (rotor+stator)
rotor = .090;
stator = 0.180;






%==========WHEEL==============


R = 0.035;      %Radius of wheels(m)


mw = .030;      %Mass of a wheel(Kg)


jw = .8*mw*R*R      %(kgm2) - Moment inertia of wheel assembly with reference to the center (axle).
                                     % Here we can assume uniform mass distribution for the wheel and the rotor, 
                                     % then work out the respective moment inertia of the wheel and rotor separately.  
                                     % The effect of gearbox is ignored.  Subsequently we add together 
                                     % the moment inertia for the wheel and rotor to get the approximate moment intertia
jrotor=.0000022

%Mw = 2*mw+2*rotor;    % Mass of 2 wheels(Kg)
%Jw = 2*jw+2*jrotor;    % MI of 2 wheels(Kgm2)
Mw=mw+rotor;
Jw=jw+jrotor;




%==========Constants============


M = Mb + 2*(Mw + Jw/R^2) - ((Mb*Lb)^2/(Mb*Lb^2 + Jb))
J = Jb + Mb*Lb^2 - (Mb*Lb)^2/(Mb + 2*(Mw+Jw/R^2))


C1 = (1/M) * (((Mb*Lb)^2*g)/(Mb*Lb^2 + Jb))
C2 = (2/M) * ( 1/R + (Mb*Lb)/(Mb*Lb^2 + Jb) )
C3 = (Mb*g*Lb)/J
C4 = (2/(J*R)) * ( R + (Mb*Lb)/(Mb + 2*(Mw + Jw/R^2)) )

%================TRANSFER_FUNCTIION=====

num = [0 -R*C4 0];
den = [C2 0 (C1*C4-C2*C3)];
G = tf(num,den)   %   g =thetaB/thetaW_dot  
                  % where i/p is angular velocity of wheel
                  % o/p is tilt angle of the bodyg = 9.8067;     %Acceleration due to gravity(m/s^2)


%================TRANSFER_FUNCTIION=====

%num = [ -R*C4 0];
%den = [C2 0 (C1*C4-C2*C3)];
%G = tf(num,den)   %   g =thetaB/thetaW_dot  
                  % where i/p is angular velocity of wheel
                  % o/p is tilt angle of the body

A = [0 1 ; (C2*C3-C1*C4)/C2 0]
B = [0 ; -R*(C4/C2)]  
D= [0 ; 0];
C =[1 0 ; 0 1];

%[A,B,C,D] = tf2ss(num,den)

% Fs = 5;
% dt = 1/Fs;
% N = 50;
% t = dt*(0:N-1);
% u = [1 zeros(1,N-1)];
% yf=filter(num,den,u)
% stem(t,yf,'o')
% x = [0;0];
% for k = 1:N
%     y(k) = C*x + D*u(k);
%     x = A*x + B*u(k);
% end
% 
% hold on
% stem(t,y,'*')
% hold off
% legend('tf','ss')