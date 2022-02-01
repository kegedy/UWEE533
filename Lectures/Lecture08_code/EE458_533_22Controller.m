%% Control design for EE 458/533 students
clc
close all
nfig = 0;
% Let us start with a simple plant
% P = 1/(s+1)

P_simple = tf(1,[1 1]);

% Let us check stability margins of uncompensated loop gain (i.e. controller =1)
% nfig=nfig+1;
% figure(nfig)
% bode(P_simple)
% grid on
% 
% % let us check step response
% nfig=nfig+1;
% figure(nfig)
% step(P_simple)

%%
% For the same plant let me change the pole location/ gain slightly
P_simple1 = tf(0.8,[1 1]);

% Let us check stability margins of uncompensated loop gain (i.e. controller =1)
nfig=nfig+1;
figure(nfig)
bode(P_simple1)
grid on

% let us check step response
nfig=nfig+1;
figure(nfig)
[resp,t]=step(P_simple1,50);
plot(t,resp)
yline(1)
% %%
% % We need it to track a dc quantity. SO a high gain at dc is needed.
Controller_P = 100;
loopgain_P = Controller_P*P_simple1;
% check stability of the compensated loop gain.
% Let us check stability margins of uncompensated loop gain (i.e. controller =1)
nfig=nfig+1;
figure(nfig)
bode(loopgain_P)
grid on

% let us check step response (closed loop now)
nfig=nfig+1;
figure(nfig)
CLTF_P = loopgain_P/(1+loopgain_P)
[resp_P,t_P]=step(CLTF_P,50);
plot(t_P,resp_P)
yline(1)
% 
% There is not a perfect tracking, so let us try to introduce an integrator
% (this has infinite gain at dc)

Controller_I = tf(1,[1 0]);
loopgain_I = Controller_I*P_simple1;
% check stability of the compensated loop gain.
% Let us check stability margins of uncompensated loop gain (i.e. controller =1)
nfig=nfig+1;
figure(nfig)
bode(loopgain_I)
grid on

% let us check step response (closed loop now)
nfig=nfig+1;
figure(nfig)
CLTF_I = loopgain_I/(1+loopgain_I)
[resp_I,t_I]=step(CLTF_I,50);
plot(t_I,resp_I)
yline(1)
