clc; clear all; close all;
load("C:\Users\franc\OneDrive\Bureaublad\codespace\Githubcode\college\Dyn and Cont\deliverable 1\MeasuredSignals.mat")
syms L_0 L_1 L_2 m_0 m_1 m_2 m_3 d_x d_w d_phi d_theta k_theta g 
syms theta_ref k_x L_x theta theta_dot theta_ddot x x_dot x_ddot
syms phi(t) phi_dot M psi F_A 


%constants
g = 9.81;
L_0 = 30;
L_1 = 25;
L_2 = 10;
m_0 = 10^4;
m_1 = 10^4;
m_2 = 2000;
m_3 = 500;
d_x = 200;
d_w = 2000;
d_phi = 2;
k_theta = 200;
theta_ref = 0;
d_theta = 10;
k_x = 7000;
L_x = 12.5;


% Generalized Coordinates
q = [x;
    theta];
q_dot = [x_dot;
    theta_dot];
q_ddot = [x_ddot;
    theta_ddot];

% Position vectors
r_2 = [x*cos(phi(t));
      L_0 + x*sin(phi(t))];

r_3 = [x*cos(phi(t)) - L_2*cos(theta);
      L_0 + x*sin(phi(t)) - L_2*sin(theta)];

r_4 = [L_1*cos(phi(t));
      L_0 + L_1*sin(phi(t))];

% Velocity vectors
r_dot_2 = jacobian(r_2, q)*q_dot + jacobian(r_2, t);
r_dot_3 = jacobian(r_3, q)*q_dot + jacobian(r_3, t);
r_dot_4 = jacobian(r_4, q)*q_dot + jacobian(r_4, t);

% Kinetic Energy
T_2 = 1/2*m_2*transpose(r_dot_2)*r_dot_2;
T_2 = simplify(T_2, 'steps', 25)
T_3 = 1/2*m_3*transpose(r_dot_3)*r_dot_3;
T_3 = simplify(T_3, 'steps', 25)
T_4 = 1/2*m_1*transpose(r_dot_4)*r_dot_4;
T_4 = simplify(T_4, 'steps', 25)
T = T_2 + T_3 + T_4;
T = simplify(T, 'steps', 25)


            % Gravitational potential energy
            V_1 = m_0*g*L_0/2;
            V_2 = m_2*g*(L_0 + x*sin(phi(t)))
            V_3 = m_3*g*(L_0 + x*sin(phi(t)) - L_2*sin(theta))
            V_4 = m_1*g*(L_0 + L_1/2*sin(phi(t)))
            
            % Elastic potential energy
            V_k_1 = 1/2*k_x*(x - L_x)^2
            V_k_2 = 1/2*k_theta*((theta - phi) - theta_ref)^2
            
            % combined
            V = (V_1 + V_2 + V_3 + V_4) + (V_k_1 + V_k_2)
            V = simplify(V)

% Damper force vector
F_dx = [-d_x*x_dot*cos(phi(t));
       -d_x*x_dot*sin(phi(t))]

Q_nc_dx = transpose(jacobian(r_2, q))*F_dx;

psi = [0;
      0;
      theta - phi(t)];

M = [0;
    0;
    -d_theta*(theta_dot - phi_dot)]

Q_nc_dtheta = transpose(jacobian(psi, q))*M;

F_W = [-d_w*theta_dot*sin(theta);
      d_w*theta_dot*cos(theta)];

Q_nc_FW = transpose(jacobian(r_3, q))*F_W;

F_A = [F_A*cos(phi(t));
      F_A*sin(phi(t))];

Q_nc_FA = transpose(jacobian(r_2, q))*F_A;

Q_nc_dx = simplify(Q_nc_dx)
Q_nc_dtheta = simplify(Q_nc_dtheta)
Q_nc_FW = simplify(Q_nc_FW)
Q_nc_FA = simplify(Q_nc_FA)

Q_nc = Q_nc_dx + Q_nc_dtheta + Q_nc_FW + Q_nc_FA;
Q_nc = simplify(Q_nc)

% First term
dTdq_dot = jacobian(T, q_dot);
First_Term = jacobian(transpose(dTdq_dot),q)*q_dot + jacobian(transpose(dTdq_dot),q_dot)*q_ddot;
First_Term = transpose(First_Term);
First_Term = simplify(First_Term)

% Second term
Second_Term = jacobian(T, q);
Second_Term = simplify(Second_Term)

% Third term
Third_Term = jacobian(V, q);
Third_Term = simplify(Third_Term)

% Equations of Motion
EoM = First_Term + Second_Term + Third_Term == transpose(Q_nc);
EoM = simplify(EoM);
EoM = transpose(EoM)

syms x theta real

% Define phi = 0
phi = 0;

% Substitute phi=0 into V
V = 1226250*sin(phi)- 49050*sin(theta) ...
    + 24525*x*sin(phi) ...
    + 3500*(x - 25/2)^2 ...
    + 100*(theta - phi)^2 ...
    + 5150250;

V = simplify(V);

% Compute gradient
dVdq = jacobian(V, [x; theta]).';

% Solve for equilibrium
sol = vpasolve( [dVdq(1) == 0, dVdq(2) == 0], ...
                [x, theta], ...
                [12.5, pi/2] );

x_0 = double(sol.x);
theta_0 = double(sol.theta);

q_0 = [x_0; theta_0]


% Find hessian and sub in equilibrium point
dVdq2 = hessian(V,q);
dVdq2 = simplify(dVdq2);
dVdq2 = subs(dVdq2,q,q_0); % sub in q0

% Determine if positive definite
eig_val = eig(dVdq2);
isposdef = all(eig_val > 0) % if =1 then stable.

M_0 = hessian(T,q_dot);
M_0 = subs(M_0,q,q_0); %sub in q = q_0
M_0 = subs(M_0,q_dot,[0;0]) % sub in q_dot = 0

D_0 = jacobian(-Q_nc,q_dot);
D_0 = subs(D_0,q,q_0);
D_0 = subs(D_0,q_dot,[0;0])

K_0 = hessian(V,q);
K_0 = subs(K_0,q,q_0);
K_0 = subs(K_0,q_dot,[0;0])

K_0_Q = jacobian(-Q_nc,q);
K_0_Q = subs(K_0_Q,q,q_0);
K_0_Q = subs(K_0_Q,q_dot,[0;0])

Q = subs(Q_nc,q,q_0);
Q = subs(Q,q_dot,[0;0])

EoM_linearized = M_0*q_ddot + D_0*q_dot + (K_0+K_0_Q)*(q-q_0) == Q


syms x(t) theta(t)
t_interval = [0,20];
y_IC = [q_0(1)+4, 0, q_0(2)+0.3, 0]; % Initial position & Initial velocity (in state space form)
    % y1 = x
    % y2 = d/dt x
    % y3 = theta
    % y4 = d/dt theta
S_desired = ["x"; "Dx"; "theta"; "Dtheta"]; % elements of state

%% Nonlinear EoM simulation calcs
EoM = subs(EoM, {'x', 'theta', 'x_dot', 'theta_dot', 'x_ddot', 'theta_ddot'}, [x(t) theta(t), diff(x,t), diff(theta,t), diff(x,t,2), diff(theta,t,2)]);
EoM = formula(EoM);
[F, S] = odeToVectorField(EoM(1),EoM(2));
S = string(S);
for i = 1:length(S_desired)
    index(i) = find(S == S_desired(i));
end
V_fun = matlabFunction(F,'vars',{'t','Y'});
y_IC = y_IC(index);
[t_sol,y_sol] = ode45(V_fun,t_interval,y_IC);
y_sol = y_sol(:,index);
x_sol = y_sol(:,1);
theta_sol = y_sol(:,3);

%{
%% Linear EoM Simulations calcs
EoM_linearized = subs(EoM_linearized, {'x', 'theta', 'x_dot', 'theta_dot', 'x_ddot', 'theta_ddot'}, [x(t) theta(t), diff(x,t), diff(theta,t), diff(x,t,2), diff(theta,t,2)]);
EoM_linearized = formula(EoM_linearized);
[F, S] = odeToVectorField(EoM_linearized(1),EoM_linearized(2));
S = string(S);
for i = 1:length(S_desired)
    index(i) = find(S == S_desired(i));
end
V_fun = matlabFunction(F,'vars',{'t','Y'});
[t_linear_sol,y_sol] = ode45(V_fun,t_interval,y_IC);
y_sol = y_sol(:,index);
x_linear_sol = y_sol(:,1);
theta_linear_sol = y_sol(:,3);

% Plot
plot(t_sol,x_sol,'r','LineWidth',2,'DisplayName','x [m]')
hold on
plot(t_linear_sol,x_linear_sol,'b--','LineWidth',2,'DisplayName','x_{linearized} [m]')
hold off
xlabel('t [seconds]')
legend()
title('Nonlinear EoM vs Linear EoM')
subtitle('x(t)')

plot(t_sol,theta_sol,'r','LineWidth',2,'DisplayName','\theta [rad]')
hold on
plot(t_linear_sol,theta_linear_sol,'b--','LineWidth',2,'DisplayName','\theta_{linearized} [rad]')
xlabel('t [seconds]')
title('Nonlinear EoM vs Linear EoM')
subtitle('\theta(t)')
legend()
%}
