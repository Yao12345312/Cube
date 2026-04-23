%% Cubli discrete LQR simulation
% mode='current': LQR + ESC current loop (firmware target)
% mode='speed'  : LQR + ESC speed loop (for comparison)
% Requires Control System Toolbox (dlqr/c2d/ss)

clear; clc;

%% Mode switch
mode = 'current';      % 'current' or 'speed'
Ts = 0.005;          % control period [s], 200 Hz
T_end = 3.0;         % simulation duration [s]
N = round(T_end / Ts);

%% Physical parameters
l = 0.0707;
l_b = 0.0623;
g = 9.7986;

m_b = 0.584;
m_w = 0.097;

k_t = 0.06;          % motor torque constant [N*m/A]
I_w = 0.12e-3;
I_b = 3.34e-3;

c_w = 0.05e-3;
c_b = 1.02e-3;

J = I_b + m_w * l * l;

% Continuous-time state: x = [theta; theta_dot; omega]
A = [0, 1, 0;
    (m_b*l_b + m_w*l)*g / J, -c_b / J, c_w / J;
   -(m_b*l_b + m_w*l)*g / J,  c_b / J, -c_w*(I_b + I_w + m_w*l*l)/(I_w*J)];

% Input is motor current i [A]
B_i = [0;
      -k_t / J;
       k_t*(I_b + I_w + m_w*l*l)/(I_w*J)];

C = eye(3);
D = zeros(3,1);

%% Common simulation setup
x = zeros(3, N+1);
x(:,1) = [0.12; 0.0; 0.0];   % initial tilt around balance point

u = zeros(1, N);

time = (0:N) * Ts;

% Limits
I_lim = 50.0;             % current loop limit [A]
omega_cmd_lim = 5000 * 2*pi/60;  % speed command limit [rad/s]

%% Mode A: LQR + ESC current loop
if strcmpi(mode, 'current')
    sysd = c2d(ss(A, B_i, C, D), Ts, 'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    % Tune Q/R as needed
    Q = diag([100, 10, 0.5]);
    R = 0.5;

    Kd = dlqr(Ad, Bd, Q, R);

    fprintf('=== Discrete LQR (current loop) ===\n');
    fprintf('Ts = %.4f s\n', Ts);
    fprintf('K(d) = [%.6f, %.6f, %.6f]\n', Kd(1), Kd(2), Kd(3));
    fprintf('Firmware formula: i_cmd = -(K*x), cmd_int = round(i_cmd*100)\n\n');

    for k = 1:N
        i_cmd = -Kd * x(:,k);
        i_cmd = max(min(i_cmd, I_lim), -I_lim);

        u(k) = i_cmd;
        x(:,k+1) = Ad * x(:,k) + Bd * i_cmd;
    end

    cmd_int = round(u * 100);

%     figure('Name','Current-loop LQR');
%     subplot(4,1,1); plot(time, x(1,:)); grid on; ylabel('theta [rad]');
%     subplot(4,1,2); plot(time, x(2,:)); grid on; ylabel('theta dot [rad/s]');
%     subplot(4,1,3); plot(time, x(3,:)); grid on; ylabel('omega [rad/s]');
%     subplot(4,1,4); stairs(time(1:end-1), cmd_int); grid on; ylabel('cmd int'); xlabel('time [s]');

%% Mode B: LQR + ESC speed loop
elseif strcmpi(mode, 'speed')
    % Inner speed loop approximation:
    % i = Kesc * (omega_cmd - omega)
    % choose tau_speed from ESC step test (63% rise time after delay)
    tau_speed = 0.03;  % [s], replace with measured value

    a33 = A(3,3);
    b3 = B_i(3);
    Kesc = (1/tau_speed + a33) / b3;

    Cw = [0 0 1];
    A_v = A - B_i * Kesc * Cw;
    B_v = B_i * Kesc;          % input: omega_cmd [rad/s]

    sysd = c2d(ss(A_v, B_v, C, D), Ts, 'zoh');
    Ad = sysd.A;
    Bd = sysd.B;

    % Optional 1-step command delay augmentation
    use_1step_delay = true;

    Q = diag([120, 12, 0.8]);
    R = 0.8;

    if use_1step_delay
        Ad_aug = [Ad, Bd;
                  0, 0, 0, 0];
        Bd_aug = [0; 0; 0; 1];

        Q_aug = diag([Q(1,1), Q(2,2), Q(3,3), 0.05]);
        Kd = dlqr(Ad_aug, Bd_aug, Q_aug, R);

        x_aug = zeros(4, N+1);
        x_aug(1:3,1) = x(:,1);

        for k = 1:N
            omega_cmd = -Kd * x_aug(:,k);
            omega_cmd = max(min(omega_cmd, omega_cmd_lim), -omega_cmd_lim);

            u(k) = omega_cmd;

            x_aug(:,k+1) = Ad_aug * x_aug(:,k) + Bd_aug * omega_cmd;
        end

        x = x_aug(1:3,:);

        fprintf('=== Discrete LQR (speed loop, 1-step delay) ===\n');
        fprintf('Ts = %.4f s, tau_speed = %.4f s\n', Ts, tau_speed);
        fprintf('K(d,aug) = [%.6f, %.6f, %.6f, %.6f]\n\n', Kd(1), Kd(2), Kd(3), Kd(4));
    else
        Kd = dlqr(Ad, Bd, Q, R);

        for k = 1:N
            omega_cmd = -Kd * x(:,k);
            omega_cmd = max(min(omega_cmd, omega_cmd_lim), -omega_cmd_lim);

            u(k) = omega_cmd;
            x(:,k+1) = Ad * x(:,k) + Bd * omega_cmd;
        end

        fprintf('=== Discrete LQR (speed loop, no delay) ===\n');
        fprintf('Ts = %.4f s, tau_speed = %.4f s\n', Ts, tau_speed);
        fprintf('K(d) = [%.6f, %.6f, %.6f]\n\n', Kd(1), Kd(2), Kd(3));
    end

    cmd_rpm = u * 60/(2*pi);

    figure('Name','Speed-loop LQR');
    subplot(4,1,1); plot(time, x(1,:)); grid on; ylabel('theta [rad]');
    subplot(4,1,2); plot(time, x(2,:)); grid on; ylabel('theta dot [rad/s]');
    subplot(4,1,3); plot(time, x(3,:)); grid on; ylabel('omega [rad/s]');
    subplot(4,1,4); stairs(time(1:end-1), cmd_rpm); grid on; ylabel('omega cmd [rpm]'); xlabel('time [s]');

else
    error('Unknown mode: use ''current'' or ''speed''.');
end
