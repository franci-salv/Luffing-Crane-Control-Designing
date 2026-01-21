% part a is H(s) stable?

% Numerator and denominator of G_I(s)
num_GI = [5.1e-3 100 250];
den_GI = [5e5 2.5e5 6.4e5 4.9e4 0];

% Transfer function G_I(s)
G_I = tf(num_GI, den_GI);

% Multiply by s
s = tf('s');
H = s * G_I;

% Display result
poles_H = pole(H);
zeros_H = zero(H);

disp('Poles of H:');
disp(poles_H);

% part b
%impulse response: y(t) due to u(t) = delta(t)
figure;
impulse(H,91)
grid on; % Adds a grid to make it easier to read
title('Impulse Response of H(s) over 91 seconds');
hold off

%Part c
% Define the three controllers
P1 = 80;
P2 = 400;
P3 = 700;


% Calculate the closed-loop transfer functions T_i(s)
% T_i(s) = feedback(P_i * G_s, 1)
T1 = feedback(P1*G_I,1);
T2 = feedback(P2*G_I,1);
T3 = feedback(P3*G_I,1);

% Analyze stability by checking the poles
poles1 = pole(T1)
poles2 = pole(T2)
poles3 = pole(T3)


% question d)  generate the step response of the closed-loop transfer function.
step(T1,240)
hold on
step(T2,240)
hold on
step(T3,240)
legend
hold off
% Calculate final value (steady-state) for each closed-loop system
final_T1 = dcgain(T1)
final_T2 = dcgain(T2)
final_T3 = dcgain(T3)



% in question d) what value do they converge to What influence does increasing P have on the final value?
% They all converge to a final value of 1
% The influence of increase P can be seen that it increases amplitude


% question e
% Define T1, T2, and T3 (REQUIRED)

Real1Min = -0.24;
Real1Max = -0.0055; 
Imag1Min = -1.2;
Imag1Max = 1.2;

Real2Min = -0.05;
Real2Max = -0.031;
Imag2Min = -0.74;
Imag2Max = 0.74;

figure;

subplot(1, 2, 1);
pzmap(T1, T2, T3);
xlim([Real1Min, Real1Max]); 
ylim([Imag1Min, Imag1Max]);
title('Zoom Level 1: Real \in [-0.24, -0.0055]');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
legend('T1', 'T2', 'T3', 'Location', 'best');

subplot(1, 2, 2);
pzmap(T1, T2, T3);
xlim([Real2Min, Real2Max]);
ylim([Imag2Min, Imag2Max]);
title('Zoom Level 2: Real \in [-0.05, -0.031]');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
legend('T1', 'T2', 'T3', 'Location', 'best');

sgtitle('Pole-Zero Map Comparison of Two Zoom Levels');
hold off

% f) C(s) = 390s+ 360
%step response: y(t) due to u(t) = 1(t)

Cs = 390*s + 360;
Tf1 = feedback(Cs*G_I,1);

step(T2,79)
hold on 
step(Tf1,79)
legend
hold off


%g) 

% --- Analysis ---
S1 = stepinfo(Tf1);
D1 = damp(Tf1);
Poles_Tf1 = pole(Tf1);

S2 = stepinfo(T2);
D2 = damp(T2);
Poles_T2 = pole(T2);

% --- Display Results (You'll need to parse D1/D2 for specific poles/frequencies) ---
disp('System 1 Results:');
disp(['Final Value (DC Gain): ' num2str(dcgain(Tf1))]);
disp(['Settling Time: ' num2str(S1.SettlingTime)]);
disp('Closed-Loop Poles:'); disp(Poles_Tf1);

disp('System 2 Results:');
disp(['Final Value (DC Gain): ' num2str(dcgain(T2))]);
disp(['Settling Time: ' num2str(S2.SettlingTime)]);
disp('Closed-Loop Poles:'); disp(Poles_T2);
% For System 2, if complex poles exist, the imaginary part of Poles_T2 is omega_d.
% E.g., Omega_d_T2 = abs(imag(Poles_T2(1))); 

% --- Plotting ---
figure;
pzmap(Tf1, T2);
axis([-0.22, -0.025, -1.2, 1.2])
grid on;
legend('Poles/Zeros Tf1', 'Poles/Zeros T2'); 
title('Poles and Zeros of Tf1 and T2');