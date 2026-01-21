clc; clear; close all;

%% (Optional) Load measured data
% load("C:\Users\franc\OneDrive\Bureaublad\codespace\Githubcode\college\Dyn and Cont\deliverable 1\MeasuredSignals.mat")

%% Symbolic generalized coordinates
syms x theta x_dot theta_dot x_ddot theta_ddot real
syms phi real    % base angle (constant parameter, NOT a function of time)

q      = [x; theta];
q_dot  = [x_dot; theta_dot];
q_ddot = [x_ddot; theta_ddot];

%% Constants (numeric)
g      = 9.81;
L_0    = 30;
L_1    = 25;
L_2    = 10;
m_0    = 1e4;
m_1    = 1e4;
m_2    = 2000;
m_3    = 500;
d_x    = 200;
d_w    = 2000;
d_phi  = 2;      % not used in current model
k_theta = 200;
theta_ref = 0;
d_theta = 10;
k_x    = 0;
L_x    = 12.5;

%% Position vectors of masses (in global frame)
r_2 = [x*cos(phi); ...
       L_0 + x*sin(phi)];

r_3 = [x*cos(phi) - L_2*cos(theta); ...
       L_0 + x*sin(phi) - L_2*sin(theta)];

r_4 = [L_1*cos(phi); ...
       L_0 + L_1*sin(phi)];

%% Velocity vectors (phi is constant => no explicit time dependence)
r_dot_2 = jacobian(r_2, q) * q_dot;
r_dot_3 = jacobian(r_3, q) * q_dot;
r_dot_4 = jacobian(r_4, q) * q_dot;   % will be zero (no dependence on x,theta)

%% Kinetic energy
T_2 = 1/2 * m_2 * (r_dot_2.' * r_dot_2);
T_3 = 1/2 * m_3 * (r_dot_3.' * r_dot_3);
T_4 = 1/2 * m_1 * (r_dot_4.' * r_dot_4);   % likely zero
T   = simplify(T_2 + T_3 + T_4);

%% Potential energy (gravity)
V_1 = m_0 * g * L_0/2;                               % base
V_2 = m_2 * g * (L_0 + x*sin(phi));                 % mass m2
V_3 = m_3 * g * (L_0 + x*sin(phi) - L_2*sin(theta));% mass m3
V_4 = m_1 * g * (L_0 + (L_1/2)*sin(phi));           % mass m1

% Elastic potential energy
V_k_1 = 1/2 * k_x * (x - L_x)^2;
V_k_2 = 1/2 * k_theta * ((theta - phi) - theta_ref)^2;

V = simplify(V_1 + V_2 + V_3 + V_4 + V_k_1 + V_k_2);

%% Non-conservative generalized forces (dampers)

% Damper in x-direction (attached to mass at r_2)
F_dx = [-d_x * x_dot * cos(phi); ...
        -d_x * x_dot * sin(phi)];
Q_nc_dx = jacobian(r_2, q).' * F_dx;

% Rotational damper between theta and phi (phi_dot = 0 here)
psi = [0; 0; theta - phi];
M_vec = [0; 0; -d_theta*(theta_dot - 0)];   % no phi_dot
Q_nc_dtheta = jacobian(psi, q).' * M_vec;

% Damper along rotation of link with theta (at mass m3)
F_W = [-d_w * theta_dot * sin(theta); ...
        d_w * theta_dot * cos(theta)];
Q_nc_FW = jacobian(r_3, q).' * F_W;

% (Optional) external actuator force along phi – set to zero for eigenfreq
% syms F_A_scalar real
% F_A_vec = [F_A_scalar*cos(phi); F_A_scalar*sin(phi)];
% Q_nc_FA = jacobian(r_2, q).' * F_A_vec;
% For free vibrations, we ignore F_A:
Q_nc = simplify(Q_nc_dx + Q_nc_dtheta + Q_nc_FW);

%% Lagrange's equations:   d/dt(dT/dqdot) - dT/dq + dV/dq = Q_nc

dT_dqdot = jacobian(T, q_dot).';   % column
dT_dq    = jacobian(T, q).';       % column
dV_dq    = jacobian(V, q).';       % column

% Time derivative of dT/dqdot (no explicit time dependence => only via q,qdot)
First_Term = jacobian(dT_dqdot, q) * q_dot + jacobian(dT_dqdot, q_dot) * q_ddot;

EoM = simplify( First_Term - dT_dq + dV_dq - Q_nc );   % = 0 (2x1 vector)

%% EQUILIBRIUM POINT (phi = 0)

phi_0 = 0;

% Potential with phi fixed
V_eq = simplify( subs(V, phi, phi_0) );

% Gradient wrt q = [x; theta]
dVdq_eq = jacobian(V_eq, q).';

% Solve dV/dq = 0 for equilibrium
sol = vpasolve( [dVdq_eq(1) == 0, dVdq_eq(2) == 0], ...
                [x, theta], ...
                [12.5, pi/2] );    % initial guess near your expected eq

x_0     = double(sol.x);
theta_0 = double(sol.theta);
q_0     = [x_0; theta_0];

disp('Equilibrium q_0 = [x_0; theta_0]:');
disp(q_0);

%% Check potential Hessian at equilibrium (for stability)

dVdq2 = hessian(V, q);
dVdq2 = simplify(dVdq2);
dVdq2 = subs(dVdq2, {x, theta, phi}, {q_0(1), q_0(2), phi_0});  % sub in q0 and phi0

eig_val  = eig(dVdq2);
isposdef = all(eig_val > 0)    % if = 1 then stable


%% Linearization matrices M_0, D_0, K_0, K_0_Q, Q

% Mass matrix from T
M_0 = hessian(T, q_dot);
M_0 = subs(M_0, {x, theta, phi, x_dot, theta_dot}, {q_0(1), q_0(2), phi_0, 0, 0})

% Damping matrix from Q_nc
D_0 = jacobian(-Q_nc, q_dot);
D_0 = subs(D_0, {x, theta, phi, x_dot, theta_dot}, {q_0(1), q_0(2), phi_0, 0, 0})

% Stiffness matrix from V
K_0 = hessian(V, q);
K_0 = subs(K_0, {x, theta, phi}, {q_0(1), q_0(2), phi_0})

% Extra stiffness from configuration-dependence of Q_nc
K_0_Q = jacobian(-Q_nc, q);
K_0_Q = subs(K_0_Q, {x, theta, phi, x_dot, theta_dot}, {q_0(1), q_0(2), phi_0, 0, 0})

% Generalized force at equilibrium (should be ~0 for free vibrations)
Q = subs(Q_nc, {x, theta, phi, x_dot, theta_dot}, {q_0(1), q_0(2), phi_0, 0, 0})


%% Linearized EoM: M_0*q_ddot + D_0*q_dot + (K_0 + K_0_Q)*(q - q_0) = Q

EoM_linearized = M_0*q_ddot + D_0*q_dot + (K_0 + K_0_Q)*(q - q_0) == Q


%% Eigenfrequencies and eigenmodes (ignore damping for this)

K_eff = K_0 + K_0_Q;
M_0_num = double(M_0);
K_eff_num = double(K_eff);

[U, lambda] = eig(M_0_num \ K_eff_num);   % generalized eigenproblem

% Mode shapes (columns)
u1 = U(:,1) ;                              % 1st eigenmode
u2 = U(:,2) ;                              % 2nd eigenmode

% Normalize so first element of each mode = 1
u1 = u1 / u1(1);
u2 = u2 / u2(1);

% Display
disp('Mode 1 (normalized so first entry = 1):');
disp(u1);

disp('Mode 2 (normalized so first entry = 1):');
disp(u2);

% Natural frequencies (rad/s)
w  = sqrt(diag(lambda));                  % vector of eigenfrequencies
w1 = w(1)                                 % 1st eigenfrequency
w2 = w(2)                                 % 2nd eigenfrequency

