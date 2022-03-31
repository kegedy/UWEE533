% Created by Rahul Mallik (02/15/2022)
% Base code :  Chapman machines book : pp 246 ELECTRIC MACHINERY FUNDAMENTALS 
% Teaching material for EE 458, 533
% Subplot 1 : Plots the three phase currents in the three coils, a, b and c
% Subplot 2 : rotating magnetic field.
% a phase current, fluxes are in black, b in red and c in blue
% 
close all
bmax = 1; % No rmalize bmax t o 1 
freq = 60; % 60 Hz 
w = 2*pi*freq; % a n g ular ve l oc ity (rad/ s ) 
% Firs t , generate the three component magnetic fi e l ds 
t = 0:1/6000:1/60; 
E_a = sin(w*t).*(cos(0)+j*sin(0));
E_b = sin(w*t - 2*pi/3)*(cos(2*pi /3) + j*sin(2*pi /3)) ; 
E_c = sin(w*t +2*pi/3)*(cos(-2*pi /3) + j*sin(- 2*pi /3)) ; 

E_net = E_a + E_b + E_c;

Baa = -cos(w*t).*(cos(0)+j*sin(0));
Bbb = -cos(w*t - 2*pi/3)*(cos(2*pi /3) + j*sin(2*pi /3)) ; 
Bcc = -cos(w*t +2*pi/3)*(cos(-2*pi /3) + j*sin(- 2*pi /3)) ; 
% Ca l c ulate Bnet 
Bnet = Baa + Bbb + Bcc; 

circle = 1.5 * (cos(w*t ) + j*sin(w*t ) ) ; 
%Plo t the magnitude and d irection of the r esulting magne ti c field



for ii= 1:length (t ) 
%plot the three phase currents
plot(circle, 'k' ) ; 
hold on
%!l; Plo t the f o ur magneti c fi e l ds 
plot( [ 0 real(E_net(ii)) ] , [ 0 imag(E_net(ii)) ] , 'k', 'LineWidth' ,3); 
plot( [0 real(Bnet(ii)) ] , [ 0 imag(Bnet(ii)) ] ,'c' ,'LineWidth',3 ) ; 
plot( [0 real(circle(ii)) ] , [ 0 imag(circle(ii)) ] ,'rx' ,'LineWidth',2 ) ; 
line([2,0],[0,0],Color = 'black',LineStyle = '--')
legend('Locus of the EMF/Flux','Back-Emf','Flux due to magnet','angle, \theta_e measured between perp-a and d axis','reference = perp-a')
axis square; 
axis( [- 2 2 -2 2 ] ) ; 

drawnow; 
pause(0.5);
hold off 
end 

