%% Mini Project: Wheel Position Control Simulation
% Cailey Cashion
% EENG 350 - SEED Lab
%
% This script runs a Simulink model of the wheel position controller
% and plots the desired position, simulated position, and motor
% voltage command.
%
% The position PI controller uses position error to calculate a
% desired wheel speed. The inner proportional velocity controller
% uses speed error to calculate the motor voltage command.
%
% Required file: Simulink_miniProject.slx
% The model must be in the current folder or on the MATLAB path.
%
% These results are simulated responses, not experimental measurements.

%% Motor Model Parameters
% The motor model relates applied voltage to angular velocity:
%
% G(s) = K*sigma / (s + sigma)

K = 2.2;
sigma = 5;

%% Controller Settings
% Kp is the proportional gain of the inner velocity controller.
% It determines how strongly the voltage command responds to the
% difference between desired and simulated wheel speed.
%
% Kp is a controller gain; it is different from the motor-model gain K.
%
% The outer position PI gains are configured in the Simulink
% controller block and are not assigned by this script.

Kp = 2.0;

%% Run the Simulink Model
% Run the model using the parameters defined above.
% Simulation duration and the position step are configured in the model.
%
% The model returns these logged signals as timeseries:
%   out.DesiredPosition - commanded wheel angle in radians
%   out.Position        - simulated wheel angle in radians
%   out.Voltage         - simulated motor voltage command in volts

out = sim('Simulink_miniProject');

%% Plot the Position Response and Motor Voltage
% The upper plot compares the desired and simulated wheel positions.
% It shows how quickly the wheel approaches its target and whether
% the response overshoots, oscillates, or retains a position error.
%
% The lower plot shows the voltage command used to produce that motion.

figure('Name', 'Mini Project: Position Control', 'Color', 'w');

% Position response.
subplot(2,1,1);

plot(out.DesiredPosition.Time, out.DesiredPosition.Data, ...
    '--', 'LineWidth', 1.5);
hold on;

plot(out.Position.Time, out.Position.Data, ...
    'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Wheel position (rad)');
title('Desired and Simulated Wheel Position');
legend('Desired', 'Simulated', 'Location', 'best');

% Motor voltage command.
subplot(2,1,2);

plot(out.Voltage.Time, out.Voltage.Data, ...
    'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Motor voltage command (V)');
title('Simulated Motor Voltage Command');
legend('Voltage command', 'Location', 'best');

%% Interpretation of Results
% For the simulation shown during testing, the desired position
% changed from zero to approximately pi radians at t = 1 second.
% This shows a half rotation of the wheel.
%
% The simulated position approached the target smoothly, with no
% overshoot or oscillation in the plotted response.
%
% The voltage command initially rose to around 6 V and then
% decreased toward zero as the wheel approached the target.
%
% This response supports stable position tracking for the simulated
% test. 
