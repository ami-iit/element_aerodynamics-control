close all; 
clear all; 
clc;

%% Data 
% (From NACA report 1292: Intensity, scale, and spectra of turbulence
% in mixing region of free subsonic jet)

x_bar = [0, 1.14, 2.29, 4.58, 7.6, 24, 32, 40];
y_bar = [1, 1.2, 1.28, 1.55, 2.0, 5, 6, 8];

x = linspace(x_bar(1), x_bar(end), 1000);
y = pchip(x_bar, y_bar, x);
y_lin = 1 + (4/24) * x;


%% PLOTS

fig = figure();
hold on;
plot(x, y, 'k-', "LineWidth", 1.5, 'DisplayName', 'Polynomial Interpolation');
plot(x, y_lin, 'b--', "LineWidth", 1.5, 'DisplayName', 'Linear approximation');
scatter(x_bar, y_bar, 24, 'filled', 'r', 'DisplayName', 'Experimental Data');


grid on;
xlim([0 40]);
ylim([0 12]);
axis square;

ylabel('$y/r$','Interpreter','latex');
xlabel('$x/r$','Interpreter','latex');
legend('Interpreter','latex','Location','nw');

saveF('nondim.pdf',[10 8])

%% P100
r = 0.049;

fig = figure();
hold on;
plot(x*r, y*r, 'k-', "LineWidth", 1.5, 'DisplayName', 'Polynomial Interpolation');
plot(x*r, y_lin*r, 'b--', "LineWidth", 1.5, 'DisplayName', 'Linear approximation');
scatter(x_bar*r, y_bar*r, 24, 'filled', 'r', 'DisplayName', 'Experimental Data');


grid on;
xlim([0 40*r]);
ylim([0 12*r]);
axis square;

ylabel('$y$ [m]','Interpreter','latex');
xlabel('$x$ [m]','Interpreter','latex');
legend('Interpreter','latex','Location','nw');

saveF('dim_P100.pdf',[10 8])

%% P220
r = 0.06;

fig = figure();
hold on;
plot(x*r, y*r, 'k-', "LineWidth", 1.5, 'DisplayName', 'Polynomial Interpolation');
plot(x*r, y_lin*r, 'b--', "LineWidth", 1.5, 'DisplayName', 'Linear approximation');
scatter(x_bar*r, y_bar*r, 24, 'filled', 'r', 'DisplayName', 'Experimental Data');


grid on;
xlim([0 40*r]);
ylim([0 12*r]);
axis square;

ylabel('$y$ [m]','Interpreter','latex');
xlabel('$x$ [m]','Interpreter','latex');
legend('Interpreter','latex','Location','nw');

saveF('dim_P220.pdf',[10 8])