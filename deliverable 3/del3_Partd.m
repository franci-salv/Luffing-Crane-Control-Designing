clc; clear; close all;

%% ================================================================
%  LINEARISED MATRICES (from your Lagrange derivation at equilibrium)
%  ================================================================

% Numerical value of sin(phi0)
phi0 = 7045509688949645/4503599627370496;
sphi = sin(phi0);

% Mass matrix M
M = [2500,      5000*sphi;
     5000*sphi, 50000];

% Damping matrix D
D = [200,       2000*sphi;
     0,         20010];

% Stiffness matrix K (K_0 + K_0_Q)
K = [0, 0;
     0, 49050*sphi + 200];

% Actuator force direction (in x-direction)
Bu = [1; 0];


%% ================================================================
%  STATE-SPACE MODEL
%  ================================================================
% State vector:
%   x_state = [ x ; theta ; x_dot ; theta_dot ]

A = [ zeros(2), eye(2);
     -M\K     , -M\D ];

B = [ zeros(2,1);
      M\Bu ];

% Outputs: x1 = x, theta1 = theta
C = [1 0 0 0;    % output X1
     0 1 0 0];   % output Theta1

Dss = zeros(2,1);

% SIMO system: one input (F_a), two outputs (x1, theta1)
sys = ss(A,B,C,Dss);


%% ================================================================
%  TRANSFER FUNCTIONS G1(s) and G2(s)
%  ================================================================
G1 = tf(sys(1));   % X1(s) / F_A(s)
G2 = tf(sys(2));   % Theta1(s) / F_A(s)
L2 = 10;

G_l = G1 + L2*G2;


%% ================================================================
%  BODE PLOTS
%  ================================================================
figure;
bode(G1);
grid on;
title('Bode Plot of G1(s) = X_1(s) / F_A(s)');

figure;
h = bodeplot(G2);                % use bodeplot so we can change options
opts = getoptions(h);
opts.PhaseWrapping = 'on';        % enable wrapping
setoptions(h, opts);

% Give MATLAB a moment to draw the plot (optional but helps)
drawnow;

% Find axes whose ylabel contains 'Phase' and set their limits
ax = findall(gcf,'Type','axes');
for k = 1:numel(ax)
    ylab = get(get(ax(k),'YLabel'),'String');
    if ischar(ylab) && contains(lower(ylab),'phase')    % detects the phase plot
        set(ax(k), 'YLim', [-180 180], 'YTick', -180:45:180);
    end
end

sgtitle('Bode Plot of G2(s) = \Theta_1(s) / F_A(s)');  % figure title
grid on;


figure;
bode(G_l);
grid on;
title('Bode Plot of G_l(s) = \G_1(s) + L_2*G_2(s)');

%% ================================================================
%  (OPTIONAL) Display transfer functions
%  ================================================================
disp('G1(s) = X1(s) / F_A(s):');
G1

disp('G2(s) = Theta1(s) / F_A(s):');
G2

%% ================================================================
%  GAIN COMPUTATION AT w2
%  ================================================================

w2 = 1.1096;   % <-- SET THIS to your assigned frequency

[mag1, ~] = bode(G1, w2);
[mag2, ~] = bode(G2, w2);
[mag3, ~] = bode(G3, w2);


mag1 = squeeze(mag1);
mag2 = squeeze(mag2);
mag3 = squeeze(mag3);


disp('--------------------------------------');
fprintf('Gain |G1(j%.3f)| = %.6f\n', w2, mag1);
fprintf('Gain |G2(j%.3f)| = %.6f\n', w2, mag2);
fprintf('Gain |G_l(j%.3f)| = %.6f\n', w2, mag3);

disp('--------------------------------------');




