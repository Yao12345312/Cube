
%% Cubli Single-Point Discrete LQR Simulation
% Based on ECC2013 Cubli paper:
% "The Cubli: A Reaction Wheel Based 3D Inverted Pendulum"
%
% Goal:
% Use paper's simplified idea:
%   1. remove unobservable yaw state
%   2. remove uncontrollable angular-momentum mode
% Then compute full 3-wheel single-point LQR gain.
%
% State definition (8 states after removing yaw):
%
% x = [ beta gamma ...      tilt angles
%       wx wy wz ...        body angular rates
%       ww1 ww2 ww3 ]'      wheel speeds
%
% Inputs:
% u = [i1 i2 i3]' motor currents
%
% Author: ChatGPT
% ------------------------------------------------------------

clear; clc; close all;

%% ==================== SAMPLE TIME ==========================
Ts    = 0.005;     % 200Hz
T_end = 5.0;
N     = round(T_end/Ts);

%% ==================== PARAMETERS ===========================
g = 9.7986;

% same parameters as your single-axis code
l   = 0.0707;
l_b = 0.0623;

m_b = 0.584;
m_w = 0.097;

k_t = 0.06;        % torque constant [Nm/A]

I_w = 0.12e-3;     % wheel inertia
I_b = 3.34e-3;     % body inertia (single-axis used value)

c_w = 0.05e-3;
c_b = 1.02e-3;

%% ==================== APPROXIMATE 3D MODEL =================
% Assume symmetric cube:
Jx = I_b + m_w*l^2;
Jy = I_b + m_w*l^2;
Jz = I_b + m_w*l^2;

J = diag([Jx Jy Jz]);

% wheel inertia matrix
Jw = diag([I_w I_w I_w]);

% gravity restoring coefficient around corner balance
%
% linearized tilt dynamics:
% torque = M*g*h * angle
%
% use same equivalent coefficient as single-axis
Kg = (m_b*l_b + m_w*l)*g;

%% ===========================================================
% States:
% [beta gamma wx wy wz ww1 ww2 ww3]
%
% beta  = pitch
% gamma = roll
%
% Kinematics near upright:
% beta_dot  = wy
% gamma_dot = wx
%
% yaw removed

A = zeros(8,8);
B = zeros(8,3);

%% angle kinematics
A(1,4) = 1;   % beta_dot = wy
A(2,3) = 1;   % gamma_dot = wx

%% body rotational dynamics
% wx_dot affected by gamma
A(3,2) = Kg/Jx;
A(3,3) = -c_b/Jx;
A(3,6) = c_w/Jx;

% wy_dot affected by beta
A(4,1) = Kg/Jy;
A(4,4) = -c_b/Jy;
A(4,7) = c_w/Jy;

% wz_dot free mode (yaw removed later weak damping only)
A(5,5) = -c_b/Jz;
A(5,8) = c_w/Jz;

%% wheel dynamics
% ww1
A(6,2) = -Kg/Jx;
A(6,3) = c_b/Jx;
A(6,6) = -c_w*(I_b+I_w+m_w*l^2)/(I_w*Jx);

% ww2
A(7,1) = -Kg/Jy;
A(7,4) = c_b/Jy;
A(7,7) = -c_w*(I_b+I_w+m_w*l^2)/(I_w*Jy);

% ww3
A(8,5) = c_b/Jz;
A(8,8) = -c_w*(I_b+I_w+m_w*l^2)/(I_w*Jz);

%% input matrix (currents)

gainx = k_t*(I_b+I_w+m_w*l^2)/(I_w*Jx);
gainy = k_t*(I_b+I_w+m_w*l^2)/(I_w*Jy);
gainz = k_t*(I_b+I_w+m_w*l^2)/(I_w*Jz);

B(:,1) = [0;0; -k_t/Jx;0;0; gainx;0;0];
B(:,2) = [0;0;0; -k_t/Jy;0;0; gainy;0];
B(:,3) = [0;0;0;0; -k_t/Jz;0;0; gainz];

%% ===========================================================
% Remove uncontrollable mode like paper:
% use ctrbf()

Co = ctrb(A,B);
rank_full = rank(Co);

fprintf('Original system order = %d\n', size(A,1));
fprintf('Controllable rank     = %d\n', rank_full);

% controllability decomposition
[Abar,Bbar,Cbar,T,k] = ctrbf(A,B,eye(8));

nc = sum(k);      % controllable states count

Ac = Abar(end-nc+1:end,end-nc+1:end);
Bc = Bbar(end-nc+1:end,:);

fprintf('Reduced controllable order = %d\n', nc);

%% ==================== DISCRETIZE ===========================
sysc = ss(Ac,Bc,eye(nc),zeros(nc,3));
sysd = c2d(sysc,Ts,'zoh');

Ad = sysd.A;
Bd = sysd.B;

%% ==================== LQR DESIGN ===========================
% Tune here

Q = diag([ ...
    300 ... angle
    300 ...
    20 ...
    20 ...
    5 ...
    1 ...
    1 ]);

% if order differs auto-fix:
Q = eye(nc);
for i=1:min(nc,7)
    vals = [300 300 20 20 5 1 1];
    Q(i,i)=vals(i);
end

R = diag([0.6 0.6 0.6]);

Kd = dlqr(Ad,Bd,Q,R);

fprintf('\n=== Single Point LQR Gain ===\n');
disp(Kd);

%% Recover full-state gain
Kfull = zeros(3,8);
Kfull(:,end-nc+1:end) = Kd;
Kfull = Kfull*T';

fprintf('\n=== Full 8-state Gain (use in firmware) ===\n');
disp(Kfull);

%% ==================== SIMULATION ===========================
sysd_full = c2d(ss(A,B,eye(8),zeros(8,3)),Ts,'zoh');

Adf = sysd_full.A;
Bdf = sysd_full.B;

x = zeros(8,N+1);

% initial corner disturbance
x(:,1) = [
    0.10;   % beta
   -0.08;   % gamma
    0;
    0;
    0;
    0;
    0;
    0];

u = zeros(3,N);

I_lim = 50;

for k=1:N

    uk = -Kfull*x(:,k);

    uk = max(min(uk,I_lim),-I_lim);

    u(:,k) = uk;

    x(:,k+1) = Adf*x(:,k) + Bdf*uk;
end

t = (0:N)*Ts;

%% ==================== PLOT ================================
figure('Name','Single Point Cubli LQR');

subplot(4,1,1);
plot(t,x(1,:),t,x(2,:),'LineWidth',1.5);
grid on;
ylabel('angle rad');
legend('\beta','\gamma');

subplot(4,1,2);
plot(t,x(3,:),t,x(4,:),t,x(5,:),'LineWidth',1.2);
grid on;
ylabel('\omega body');

subplot(4,1,3);
plot(t,x(6,:),t,x(7,:),t,x(8,:),'LineWidth',1.2);
grid on;
ylabel('\omega wheel');

subplot(4,1,4);
stairs(t(1:end-1),u');
grid on;
ylabel('Current A');
xlabel('Time s');

%% ==================== EXPORT FOR MCU =======================
fprintf('\nKfull rows = motor1 motor2 motor3\n');
fprintf('state order:\n');
fprintf('[beta gamma wx wy wz ww1 ww2 ww3]\n\n');

for r = 1:3
    fprintf('Motor %d:\n',r);
    fprintf('%.6f ',Kfull(r,:));
    fprintf('\n\n');
end

