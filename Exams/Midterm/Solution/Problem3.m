% problem 3 midterm solution
% Choose your own p
clc
close all
clear all
nfig = 0;
p = 200;

Gp1 = tf(0.35*p,[4/(3*p) 1]);
Gp2 = tf(1,[1/(5*p) 1]);
Gp3 = tf(1,[5/p^2 1]);

Gp = Gp1*Gp2*Gp3;

nfig = nfig+1;
figure(nfig)
bode(Gp)
grid on

% Convert time domain characteristics to freq domain
max_overshoot = 0.4 ;%40% max, if vref =10, max v = 14
t_rise  = 10e-3;% 1 ms
syms zeta positive real
sol = double(solve(exp(-pi*zeta/sqrt(1-zeta^2))==max_overshoot))
PM = atan(2*sol/sqrt(1-2*sol^2))*(180/pi)
wc = 2.5/t_rise
% PM is the minimum phase margin in degrees
% wc is the minimum gain crossover frequency

%% Control target 1: Make infinite gain at dc: Add an integrator
Control_I = tf(1,[1 0]);
%loop gain with integral control

loop_gain_I = Gp*Control_I;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_I)
grid on

nfig = nfig+1;
figure(nfig)
step(loop_gain_I/(1+loop_gain_I))
grid on

%% What did we lose? The bandwidth became really small = 64.2 rad/s
% How to improve? Multiply with a constant number to pull the gain higher.
% Calculation : I want a bandwidth of  wc
% So multiplying factor, K_gain = fs*2*pi/10/227

k_I = wc/64.2; % integral gain
Control_I_2 = tf(k_I,[1 0]);
%loop gain with integral control with gain

loop_gain_I_2 = Gp*Control_I_2;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_I_2)
grid on


%% Now we see that the system is no longer stable. Reason: There is no Phase margin
% SOlution : Add a zero.
% Where to add the zero? Just after the gain crossover freq.

Control_PI = Control_I_2*tf([1/200 1],1);

%loop gain with proportional-integral control with gain
loop_gain_PI = Gp*Control_PI;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_PI)
grid on

nfig = nfig+1;
figure(nfig)
step(loop_gain_PI/(1+loop_gain_PI))
grid on

