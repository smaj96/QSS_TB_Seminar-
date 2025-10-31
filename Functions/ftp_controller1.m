function ctl_out = ftp_controller1(input)


%comment by shaker at 31,10,2025 pulled 
%pulled
%answer by smaj96 : wet56myv make branch to modify the push!!! 2nd.
w_MGB = input(1);            % Angular velocity at torque combiner/gearbox
dw_MGB = input(2);           %  angular acceleration (rate of change of W_MGB)
T_MGB = input(3);            % Total torque demand at the gearbox input 
Q_BT = input(4);            % Battery charge[C]
%i = input(4);            % 4
%v = input(5);            % 5

global w_EM_max;             % maximum motor angular velocity [rads/s](global) 
global T_EM_max;             %  maximum motor torque[Nm] (global)
theta_EM = 0.1;              %  motor rotor inertia[kg.m2]

T_MGB_th = 35;              % define torque threshold for NEDC 

eps = 0.01;              % small buffer value for numerical robustness


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Parameters ---
Q_min = 3600;        % Minimum allowable battery charge [C] 
Q_max = 32400;  %15     % Maximum allowable battery charge [C]
Q_low  = Q_min + 1;  % Hysteresis buffer to prevent rapid switching 
T_thresh = 32; %16.5   % Threshold for electric-only mode
u_LPS_max = 0.001;  % Max torque split ratio in LPS
dw_thresh = 0.1;  % Optional: acceleration threshold

% --- Default outputs ---
state_CE = 0;
u = 0;
% --- Propulsion ---
if T_MGB > 0
    if Q_BT > Q_low
        if T_MGB < T_thresh
            % Electric-only drive
            state_CE = 0;
            u = 1;
        else
            % Hybrid mode (LPS)
            state_CE = 1;
            if Q_BT < Q_max
                u = min((interp1(w_EM_max, T_EM_max, w_MGB) - abs(theta_EM * dw_MGB) - eps) / T_MGB, u_LPS_max); %% added by shak 28/6
            else
                u = 0;
            end
        end
    else
        % Low charge → engine only
        state_CE = 1;
        u = 0;
    end

% --- Regeneration ---
elseif T_MGB < 0
    if Q_BT < Q_max
        u = min((interp1(w_EM_max, -T_EM_max, w_MGB) + abs(theta_EM * dw_MGB) + eps) / T_MGB, 0.998);
    else
        u = 0;  % Friction brake fallback
    end
    state_CE = 0;

% --- Coasting / Idle ---
else
    state_CE = 0;
    u = 0;
end

ctl_out = [state_CE, u];




