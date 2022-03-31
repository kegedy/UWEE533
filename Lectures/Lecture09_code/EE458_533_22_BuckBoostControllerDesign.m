% Buck-boost inductor current control
% Parameters
clc
close all

nfig = 0;
fs = 50e3;
Vin = 24;
T = 1/fs;
L = 230e-6;
C = 200e-6;
P = 400;
D_nominal = 0.75;
Vo_nominal = Vin*D_nominal/(1-D_nominal);
R = Vo_nominal^2/P;
IL_nominal = Vo_nominal/(R*(1-D_nominal));

G_id = tf([C*(Vin+Vo_nominal) ((Vin+Vo_nominal)/R)+(1-D_nominal)*IL_nominal],[L*C L/R (1-D_nominal)^2])

%% Check if the open loop is stable

nfig = nfig+1;
figure(nfig)
bode(G_id)
grid on

% Yes, it is stable in open loop

%% Control target 1: Make infinite gain at dc: Add an integrator
Control_I = tf(1,[1 0]);
%loop gain with integral control

loop_gain_I = G_id*Control_I;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_I)
grid on

%% What did we lose? The bandwidth became really small = 227 rad/s
% How to improve? Multiply with a constant number to pull the gain higher.
% Calculation : I want a bandwidth of 1/10th of switching frequency =
% fs*2*pi/10
% So multiplying factor, K_gain = fs*2*pi/10/227

wgi = 2*pi*fs/10; % desired bandwidth
k_I = wgi/227; % integral gain
Control_I_2 = tf(k_I,[1 0]);
%loop gain with integral control with gain

loop_gain_I_2 = G_id*Control_I_2;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_I_2)
grid on

%% Now we see that the system is no longer stable. Reason: There is no Phase margin
% SOlution : Add a zero.
% Where to add the zero? Just after the gain crossover freq.

Control_PI = Control_I_2*tf([1/9e3 1],1);

%loop gain with proportional-integral control with gain
loop_gain_PI = G_id*Control_PI;
nfig = nfig+1;
figure(nfig)
bode(loop_gain_PI)
grid on


%% Boost converter

fs = 50e3;
Vin = 24;
T = 1/fs;
L = 0.2e-3;
C = 2200e-6;
P = 200;
rL = 60e-3;
D = 0.5;
Dp = 0.5;
Vout = Vin/Dp;
R = Vout^2/P;
Gid0 = 2*Vout/(rL+Dp^2*R);
wzi = 2/(R*C);
w0 = sqrt((1/(L*C))*((rL/R) + Dp^2));
Q = w0/((rL/L)+(1/(R*C)));
Gid = Gid0*tf([1/wzi 1],[1/w0^2 1/(Q*w0) 1]);

% AT high freq, approx this by letting s tends to infty
Gid_approx = (Gid0*w0^2/wzi)*tf(1,[1 0]);
%% Check if the open loop is stable

nfig = nfig+1;
figure(nfig)
bode(G_id)
hold on 
bode(Gid_approx)
grid on
