clear all
clc

% This is an example script for buck converter. 
% The numbers here will be different from the actual assignment.
% It is up to you to modify this for your homework.

% Parameters
fs = 50e3; 
T = 1/fs;
L = 150e-6;
C = 10e-6;
R = 10;
Ron = 15e-3;

% Input values before and after step changes
vg_before = 75;
vg_after = 100;
tStepVg = 40e-3;

d_before = 0.75;
d_after = 0.5;
tStepDuty = 20e-3;

% In part (c) we get the ss values below
Vg = vg_before;
D = d_before;
I = D*Vg/R;
V = D*Vg;


%% code below runs simulation and plots result

tStop = 60e-3;

[t_sw, x_sw, y_sw] = sim('Circuit1_Switched', tStop);

i_sw = x_sw(:,1);
v_sw = x_sw(:,2);

[t_avg, x_avg, y_avg] = sim('EE458_HW1_Circuit1_Averaged', tStop);

i_avg = x_avg(:,1);
v_avg = x_avg(:,2);

%%

close all

for i = 1:3
    
    figure (i)
    
a1 = subplot(2,1,1);
plot(t_sw, v_sw, 'LineWidth', 1)
hold on
plot(t_avg, v_avg, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$v$, [V]','Interpreter','latex'); 
legend('give meaningul label', 'give meaningul label')
title('give meaningul title')

a2 = subplot(2,1,2);
plot(t_sw, i_sw, 'LineWidth', 1)
hold on 
plot(t_avg, i_avg, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$i$, [A]','Interpreter','latex'); 
legend('give meaningul label', 'give meaningul label')

if i == 1
    linkaxes([a1 a2],'x')
    xlim([0, tStop])
elseif i == 2
    linkaxes([a1 a2],'x')
    xlim([tStepDuty - 5*T, tStepDuty + 25*T])
else
    linkaxes([a1 a2],'x')
    xlim([tStepVg - 5*T, tStepVg + 25*T])
end

end
