clc; clear; close all;

control_data = readmatrix('poke_break_logged_data.csv');

timestamps = control_data(:, 1);
motor_commanded_torque = control_data(:, 2);
motor_encoder_torque = control_data(:, 3);
motor_encoder_velocity = control_data(:, 6);

velocity_limit = 5; % revs / second

figure
plot(timestamps, motor_commanded_torque);
xlabel('')
ylabel('Nm')


figure
plot(timestamps, motor_encoder_velocity);
yline('r--')
xlabel('')
ylabel('Nm')
