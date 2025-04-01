%{
Author(s): Graeme Appel and Ethan Andrews with collaborations and derivations by: Owen Storer and Nathan Brasier

Project: Dynamics Lab -- Lab 3 -- Rotary Control Arm

Creation Date: 4/1/25

Last Updated: 4/1/25

Inputs:

K_g   Gear Ratio --> proportional constant relating angular velocity of the motor to the output shaft
K_m   Motor Constant --> Proportionality constant between current input to motor and torque it produces
J   Moment of Inertia About the shaft --> combination of hub inertia and inertia due to added load
K_ptheta  % Proportionality constant for Proportional component of error in PD controller
K_Dtheta % Proportionality Constant for Derivative component of error in PD controller
R_m  % Output Motor Resistance
theta_L  % Theta Actual --> current angular position of control arm
theta_D  % Theta desired --> desired angular position of control arm
s  % Laplace Transform variable for characteristic equation
   
Outputs: 
   
Purpose: 

%}

%% Housekeeping
clc
clear
close all

%% Rigid Dynamics

%% Define Necessary Variables

K_g =  33.3; % unitless
K_m = 0.0401; % unitless
R_m = 19.2; % in [Ohms] 
J_hub = 0.0005; % in [Kgm^2]
J_extra = 0.2*0.2794^2; % in [Kgm^2]
J_load = 0.0015; % in [Kgm^2]
J_total = J_hub + J_extra + J_load; % in [Kgm^2]

K_ptheta = [10, 20, 5, 10, 10, 10]; % Vector of K1-Proportional constants for Sets 1-6
K_Dtheta = [0, 0, 0, 1 , -1, -0.5]; % Vector of K3-Derivative constants for Sets 1-6
numsets = 6;

% Equation 17 Definition for reference
% [Eqn 17] -> theta_L/theta_D = ((K_ptheta*K_g*K_m)/(J*R_m))/((s^2)+((K_g^2*K_m^2)/(J*R_m)+(K_Dtheta*K_g*K_m)/(J*R_m))*(s) + (K_ptheta*K_g*K_m)/(J*R_m))) 


%% Defining Coefficients for Transfer Function System


for i = 1:numsets
n1(i) = (K_ptheta(i)*K_g*K_m)/(J_total*R_m); % Numerator Coefficient
d2(i) = 1; % Coefficient for s^2
d1(i) = ((K_g^2*K_m^2)/(J_total*R_m)+(K_Dtheta(i)*K_g*K_m)/(J_total*R_m)); % Coefficient for s
d0(i) = (K_ptheta(i)*K_g*K_m)/(J_total*R_m); % coefficient for s^0 i.e 1

 %% Closed Loop System Definition

num = n1(i); % coefficient numerator to be input into transfer function system
den = [d2 d1 d0]; % coefficients for orders of s in denominator to be input into transfer function system
sysTF = tf(num,den); % System Defintion with numerator and denominator coefficients



%% Step Response

% Exploring Base results with step function

[x,t] = step(sysTF); % Behind the scenes, step function multiplies by 1/s (i.e t in laplace transform)
                     % Steps through system defined with unit step of 1 radian --> later we will need to convert to a 0.5 unit step by multiplying x by 0.5
x = 0.5*x; % convert to unit step of 0.5 radians


% Plot Output Step values versus time
figure();
plot(t,x);
title("Rotary Arm Step Response")
xlabel("Time in [s]")
ylabel("Actual Output/Theta Value in [Rads]")

end



% Exploring More accurate results with lsim function --> lsim gives more control to input to the system


