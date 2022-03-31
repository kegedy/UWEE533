%% Buck boost converter voltage control with plant TF where voltages are positive
clc
close all
opts = bodeoptions;
opts.Title.FontSize = 14;
opts.FreqUnits = 'Hz';
opts.PhaseWrapping = 'on';
opts.Grid = 'on';


nfig = 0;
fs = 200e3;
Vin = 48;
T = 1/fs;
L = 50e-6;
rL = 100e-3;
C = 220e-6;
R = 5;
Voref = 15;
% We know for buck boost according to sign convention in class,
% Vo = DVin/(1-D)
% we can solve it to find the nominal duty ratio
syms dsym positive

D = double(solve(Vin*dsym/(1-dsym+ (rL/(R*(1-dsym))))==Voref, dsym<0.5))

Dp = 1-D;

I = Voref/(Dp*R);


% Obtain the transfer function as derived in lecture with rL = 0
Gd0 = (Vin+Voref)/Dp;

wz = Dp*(Vin+Voref)/(L*I);

w0 = Dp/sqrt(L*C);

Q = Dp*R*sqrt(C/L);

Gvd_wo_rL = Gd0*tf([-1/wz 1],[1/w0^2 1/(Q*w0) 1])


% Obtain the transfer function with rL not eq 0

Amatrix = [-rL/L      -Dp/L;
    Dp/C        -1/(R*C)];

Bmatrix = [(Vin+Voref)/L      D/L;
     -I/C               0 ];
 
Cmatrix = [0 1];

%[b,a] = ss2tf(A,B,C,D,ni) returns the transfer function that results 
% when the nith input of a system with multiple inputs is excited by a unit impulse.
% We want the output for the 1st input, the duty ratio.
% Also D = 0 

[num,den] = ss2tf(Amatrix,Bmatrix,Cmatrix,[0 0],1)

Gvd_w_rL = tf(num,den)


nfig=nfig+1;
figure(nfig)
opts.Title.String= "Plant transfer function : Gvd";
bode(Gvd_wo_rL, opts)
hold on 
bode(Gvd_w_rL, opts)
grid on
legend('Without ESR','With ESR')

% moving on, we will use the one with esr

%% Add integral gain and observe changes

C_I = tf(1,[1,0]);

loopgain_I = Gvd_w_rL*C_I;

nfig=nfig+1;
figure(nfig)
opts.Title.String= "Plant transfer function : Gvd with integral control";
bode(Gvd_w_rL, opts)
hold on
bode(loopgain_I, opts)
grid on
legend('PLant TF','Plant TF and integrator control')

%% Add integral gain, observe changes
% the current gain crossover is 12.7 Hz. Let us make that 1/20 times
% switching frewq = 10kHz. So multiplying factor = 10kHz/12.7Hz = 

k_I = 10e3/12.7; % integral gain
Control_I_2 = tf(k_I,[1 0]);
%loop gain with integral control with gain

loop_gain_I_2 = Gvd_w_rL*Control_I_2;
nfig = nfig+1;
figure(nfig)
opts.Title.String= "Plant transfer function : Gvd with integral control with gain";
bode(Gvd_w_rL, opts)
hold on
bode(loop_gain_I_2,opts)
grid on
legend('PLant TF','Plant TF and integrator control and gain')

%% Integrator, gain and pole
% System has a negative gain and phase margin problem.
% First attempt : add a zero just after gain crossover freq : 2.56e3 Hz

Control_PI = Control_I_2*tf([1/(2*pi*3e3) 1],1);

% Control_PI = Control_I_2*tf(1,[1/(2*pi*1e3) 1])*tf(1,[1/(2*pi*1e3) 1]);

%loop gain with proportional-integral control with gain
loop_gain_PI = Gvd_w_rL*Control_PI;
nfig = nfig+1;
opts.Title.String= "Plant transfer function : Gvd with integral control with gain and zero";
figure(nfig)
bode(Gvd_w_rL, opts)
hold on
bode(loop_gain_PI,opts)
grid on
legend('PLant TF','Plant TF and integrator control, gain and zero')


%% Add lead compensator
f_compens = 2.81e3;
angle_compens = 60*pi/180;

fz = f_compens*sqrt((1-sin(angle_compens))/(1+sin(angle_compens)))
fp = f_compens*sqrt((1+sin(angle_compens))/(1-sin(angle_compens)))

Control_PI_lead = Control_PI*tf([1/(2*pi*fz) 1],[1/(2*pi*fp) 1]);

%loop gain with proportional-integral control with gain
loop_gain_PI_lead = Gvd_w_rL*Control_PI_lead;
nfig = nfig+1;
opts.Title.String= "Plant transfer function : Gvd with integral control with gain and zero and lead";
figure(nfig)
bode(Gvd_w_rL, opts)
hold on
bode(loop_gain_PI_lead,opts)
grid on
legend('PLant TF','loop gain with PI and lead')

%% Add another lead compensator
% I already have 21 deg PM, I need 52 deg PM
f_compens = 6.12e3;
angle_compens = (40-21)*pi/180;

fz1 = f_compens*sqrt((1-sin(angle_compens))/(1+sin(angle_compens)))
fp1 = f_compens*sqrt((1+sin(angle_compens))/(1-sin(angle_compens)))

Control_PI_lead_square = Control_PI_lead*tf([1/(2*pi*fz1) 1],[1/(2*pi*fp1) 1]);

%loop gain with proportional-integral control with gain
loop_gain_PI_lead_square = Gvd_w_rL*Control_PI_lead_square;
nfig = nfig+1;
opts.Title.String= "Plant transfer function : Gvd with integral control with gain and poles and lead^2";
figure(nfig)
bode(Gvd_w_rL, opts)
hold on
bode(loop_gain_PI_lead_square,opts)
grid on
legend('PLant TF','loop gain with PI and lead^2')

%% Final adjustment reduce bandwidth slightly
% I already have 21 deg PM, I need 52 deg PM
k_dec = 0.6;

Control_PI_lead_square_dec = Control_PI_lead_square*k_dec;

%loop gain with proportional-integral control with gain
loop_gain_PI_lead_square_dec = Gvd_w_rL*Control_PI_lead_square_dec;
nfig = nfig+1;
opts.Title.String= "Plant transfer function : Gvd with integral control with gain and poles and lead^2 and bandwdith reduction";
figure(nfig)
bode(Gvd_w_rL, opts)
hold on
bode(loop_gain_PI_lead_square_dec,opts)
grid on
legend('PLant TF','loop gain with PI and lead^2')

CLTF = loop_gain_PI_lead_square_dec/(1+loop_gain_PI_lead_square_dec);
nfig = nfig+1;
figure(nfig)
step(CLTF)
grid on
% %% Reduce bandwidth slightly to avoid the -180 to 180 phase transition due to resonant pole of the plant
% % We realize that we have to add too many phase lead compenstors to make
% % this stable. We can do that, instead, easier option is to reduce the gain
% % crossover before all that phase jump happens.
% % Phase jump happens at 1.26 kHz, the current gain cross over is at 2.81
% % kHz
% % So we need to multiply the plot by a gain <1.26/1.83 = 0.6885 to reduce gain
% % crossover freq
% k_red = 0.03;
% 
% Control_PI_red = k_red*Control_I_2*tf([1/(2*pi*3e3) 1],1);
% 
% %loop gain with proportional-integral control with gain
% loop_gain_PI_red = Gvd_w_rL*Control_PI_red;
% nfig = nfig+1;
% opts.Title.String= "Plant transfer function : Gvd with integral control with gain,zero and reduction of gain";
% figure(nfig)
% bode(Gvd_w_rL, opts)
% hold on
% bode(loop_gain_PI_red, opts)
% grid on
% legend('PLant TF','Plant TF and integrator control, gain,zero and red')
% 
% nfig=nfig+1;
% figure(nfig)
% pzmap(loopgain_I_zero/(1+loopgain_I_zero))
% grid on