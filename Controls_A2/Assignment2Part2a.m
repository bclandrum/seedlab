clear;
clc;

% Read file
results = readtable('ExperimentalResults2a.csv');

time = results.Time_s;
voltage = results.Voltage_V;
speed = results.Speed_rad_s;

% Create one figure with both plots
figure('Color', 'w');

% Voltage is estimated from the duty cycle
subplot(2,1,1);
stairs(time, voltage, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Experimental Motor Voltage');
xlim([0 5]);

% Plot the encoder speed 
subplot(2,1,2);
plot(time, speed, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Experimental Motor Speed');
xlim([0 5]);

% Keep the zero-speed line visible
if all(speed == 0)
    ylim([-0.1 0.1]);
end
