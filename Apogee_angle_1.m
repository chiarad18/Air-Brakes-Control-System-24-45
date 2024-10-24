clc; clear; close all;

g = 9.81; % m/s^2
Cd = 0.95; % unitless
A = 0.0012; % m^2

Cf = 0.3; % unitless
A2 = 0.001;
m = 16.45; % kg
rho = 1.293; % kg/m^3

v_0 = 170; % m/s
h_0 = 378; % m
angle = 2; %deg (from the vertical)

dt = 0.1; % Time step

actuation_values = 0:0.25:1; % Actuation values from 0% to 100%

figure;
hold on;

for actuation = actuation_values
    v = v_0;
    v_y = v_0 * cosd(angle);
    h = h_0;
    t = 0;
    n = 1;
    
    a_total = [];
    a_y_total = [];
    v_total = [];
    v_y_total = [];
    h_total = [];
    t_size = [];
    F_total = [];
    
    while v_y > 0
        F_p = 0.5 * Cd * A * v^2 * rho * 4 * actuation; % Drag force with actuation

        % F_f = 0.5 * Cf * A2 * v^2 * rho; % Drag force with actuation

        F_total(n) = F_p + m*g; % Store drag force
        
        a = (-m * g - F_p) / m; % Net acceleration
        
        a_y = (-m * g - F_p*cosd(angle)) / m; % Vertical accerleration

        v = v + a * dt; % Update velocity
        
        v_y = v_y + a_y * dt; % Update vertical velocity

        h = h + v_y * dt; % Update height

        % Store values for plotting
        a_total(n) = a;
        a_y_total(n) = a_y;
        v_total(n) = v;
        v_y_total(n) = v_y;
        h_total(n) = h;
        t_size(n) = t;

        t = t + dt; % Increment time
        n = n + 1;
    end

    % Plot altitude for this actuation
    plot(t_size, h_total, 'DisplayName', sprintf('Actuation %.0f%%', actuation * 100));
end

xlabel('Time (s)');
ylabel('Altitude (m)');
legend('show')
title('Altitude vs Time for Different Actuations');

disp(max(h_total) * 3.28084); % Convert maximum height to feet

   