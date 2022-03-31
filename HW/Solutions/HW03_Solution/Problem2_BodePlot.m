Vin = 24;
Vout = 48;
Pout = 250;
fsw = 100000;
L = 1.25e-3;
C = 250e-6;
Dp = 0.5;
R = 9.216;

Go = Vout/Dp;
wz = ((Dp^2)*R)/L;
wo = Dp/sqrt(L*C);
Q = Dp*R*sqrt(C/L);

% original
sys = tf([-Go/wz Go],[1/wo^2 1/(Q*wo) 1]);

% (a) L increases
L = 5e-3; % update L

Go = Vout/Dp;
wz = ((Dp^2)*R)/L;
wo = Dp/sqrt(L*C);
Q = Dp*R*sqrt(C/L);
sysL = tf([-Go/wz Go],[1/wo^2 1/(Q*wo) 1]);

% (b) Rnew = 23.04
L = 1.25e-3; % original L
R = 23.04; % update R

Go = Vout/Dp;
wz = ((Dp^2)*R)/L;
wo = Dp/sqrt(L*C);
Q = Dp*R*sqrt(C/L);
sysRn = tf([-Go/wz Go],[1/wo^2 1/(Q*wo) 1]);

% (d) D' new and R newnew
R = 3.6; % update R
Dp = 0.8; % update Dprime
Vout = 30;

Go = Vout/Dp;
wz = ((Dp^2)*R)/L;
wo = Dp/sqrt(L*C);
Q = Dp*R*sqrt(C/L);
sysRnn = tf([-Go/wz Go],[1/wo^2 1/(Q*wo) 1]);

plotoptions = bodeoptions;
plotoptions.Grid = 'on';
bodeplot(sys,'--',sysL,'b',sysRn,'r',sysRnn,'k',plotoptions)
legend({'original','L increase','R new','R&Dp new'},'FontSize',14);

[Gm_orginal,Pm_original] = margin(sys)
[Gm_a,Pm_a] = margin(sysL)
[Gm_b,Pm_b] = margin(sysRn)
[Gm_d,Pm_d] = margin(sysRnn)
GMdB_orginal = mag2db(Gm_orginal)
GMdB_a = mag2db(Gm_a)
GMdB_b = mag2db(Gm_b)
GMdB_d = mag2db(Gm_d)







