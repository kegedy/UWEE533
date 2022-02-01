clear all
clc

% Parameters
fs = 25e3; 
T = 1/fs;
L = 300e-6;
C = 10e-6;
R = 10;
Ron = 15e-3;
n = 2; % for problem 2 only

% Input values before and after step changes
vg_before = 100;
vg_after = 75;
tStepVg = 20e-3;

d_before = 0.5;
d_after = 0.75;
d_beforeProblem1 = 0.7;
d_afterProblem1 = 0.9;
tStepDuty = 40e-3;

% In part (c) we get the ss values below
Vg = vg_before; % value of v_g(t) evaluated at t = 0
D = 1/2; % value of d(t) evaluated at t = 0

% Define linearized model matrices from part (d)
U = [D; Vg]; %same for both problems

% problem 1
I1 = 0;
V1 = 0;
A1 = [-2*Ron/L -1/L; 1/C -1/(R*C)];
B1 = [2*Vg/L (2*D-1)/L; 0 0];
X1 = [I1; V1];

% problem 2
X2 = n*D*Vg*[n; (1-D)*R]/(D*n^2*Ron + R*(1 - D)^2);
I2 = X2(1);
V2 = X2(2);
A2 = [-D*Ron/L -(1-D)/(n*L); (1-D)/(n*C) -1/(R*C)];
B2 = [(Vg - I2*Ron + V2/n)/L D/L; -I2/(n*C) 0];

% code below runs simulations and plots results

tStop = 60e-3;

%% run problem 1 simulations

 [t_sw, x_sw, y_sw] = sim('Problem1_Switched', tStop);

i_sw_1 = x_sw(:,1);
v_sw_1 = x_sw(:,2);

[t_avg, x_avg, y_avg] = sim('Problem1_Averaged', tStop);

i_avg_1 = x_avg(:,1);
v_avg_1 = x_avg(:,2);

[t_ss, x_ss, y_ss] = sim('Problem1_Linearized', tStop);

i_ss_1 = x_ss(:,1);
v_ss_1 = x_ss(:,2);

%% plot problem 1

close all

for i = 1:3
    
    figure (i)
    
a1 = subplot(2,1,1);
plot(t_sw, v_sw_1, 'LineWidth', 1)
hold on
plot(t_avg, v_avg_1, 'LineWidth',2)
hold on
plot(t_ss, v_ss_1, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$v$, [V]','Interpreter','latex'); 
legend('switched', 'averaged', 'linearized')
title('problem 1')

a2 = subplot(2,1,2);
plot(t_sw, i_sw_1, 'LineWidth', 1)
hold on 
plot(t_avg, i_avg_1, 'LineWidth',2)
hold on 
plot(t_ss, i_ss_1, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$i$, [A]','Interpreter','latex'); 
legend('switched', 'averaged', 'linearized')

if i == 1
    linkaxes([a1 a2],'x')
    xlim([0, tStop])
elseif i == 2
    linkaxes([a1 a2],'x')
    xlim([tStepVg - 5*T, tStepVg + 25*T])
else
    linkaxes([a1 a2],'x')
    xlim([tStepDuty - 5*T, tStepDuty + 25*T])
end

end

%% run problem 2 simulations

[t_sw, x_sw, y_sw] = sim('Problem2_Switched', tStop);

i_sw_2 = x_sw(:,1);
v_sw_2 = x_sw(:,2);

[t_avg, x_avg, y_avg] = sim('Problem2_Averaged', tStop);

i_avg_2 = x_avg(:,2);
v_avg_2 = x_avg(:,1);

[t_ss, x_ss, y_ss] = sim('Problem2_Linearized', tStop);

i_ss_2 = x_ss(:,1);
v_ss_2 = x_ss(:,2);

%% plot problem 2

for i = 4:6
    
    figure (i)
    
a1 = subplot(2,1,1);
plot(t_sw, v_sw_2, 'LineWidth', 1)
hold on
plot(t_avg, v_avg_2, 'LineWidth',2)
hold on
plot(t_ss, v_ss_2, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$v$, [V]','Interpreter','latex'); 
legend('switched', 'averaged', 'linearized')
title('problem 2')

a2 = subplot(2,1,2);
plot(t_sw, i_sw_2, 'LineWidth', 1)
hold on 
plot(t_avg, i_avg_2, 'LineWidth',2)
hold on 
plot(t_ss, i_ss_2, 'LineWidth',2)
xlabel('$t$, [s]','Interpreter','latex'); 
ylabel('$i$, [A]','Interpreter','latex'); 
legend('switched', 'averaged', 'linearized')

if i == 4
    linkaxes([a1 a2],'x')
    xlim([0, tStop])
elseif i == 5
    linkaxes([a1 a2],'x')
    xlim([tStepVg - 5*T, tStepVg + 25*T])
else
    linkaxes([a1 a2],'x')
    xlim([tStepDuty - 5*T, tStepDuty + 25*T])
end

end
