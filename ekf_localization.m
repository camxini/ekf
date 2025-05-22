function [] = ekf_localization()
 
% EKF
% Jiaxin Yang
% 7 Apr, 2025

    close all;
    clear all;

    disp('EKF Start!')

    time = 0;
    global endTime; % [sec]
    endTime = 60;
    global dt;
    dt = 0.1; % [sec]

    removeStep = 5;

    nSteps = ceil((endTime - time)/dt);

    estimation.time=[];
    estimation.u=[];
    estimation.GPS=[];
    estimation.xOdom=[];
    estimation.xEkf=[];
    estimation.xTruth=[];

    % State Vector [x y yaw]'
    xEkf=[0 0 0]';
    PEkf = eye(3);

    % Ground True State
    xTruth=xEkf;

    % Odometry Only
    xOdom=xTruth;

    % Observation vector [x y yaw]'
    z=[0 0 0]';

    % Simulation parameter
    global noiseQ
    noiseQ = diag([0.1 0 degreeToRadian(10)]).^2; %[Vx Vy yawrate]

    global noiseR
    noiseR = diag([0.5 0.5 degreeToRadian(5)]).^2;%[x y yaw]
    
    % Covariance Matrix for motion
%     convQ=?
    %noiseQ_avg = mean(noiseQ, 1);
    %noiseQ_centered = noiseQ - noiseQ_avg;
    %convQ = (noiseQ_centered' * noiseQ_centered) / 2;
    convQ = noiseQ;

    % Covariance Matrix for observation
%     convR=?
    %noiseR_avg = mean(noiseR, 1);
    %noiseR_centered = noiseR - noiseR_avg;
    %convR = (noiseR_centered' * noiseR_centered) / 2;
    convR = noiseR;

    % Other Intial
    % ?
    % xEkf = xOdom;
    % PEkf = eye(3);

    % Main loop
    for i=1 : nSteps
        time = time + dt;
        % Input
        u=robotControl(time);
        % Observation
        [z,xTruth,xOdom,u]=prepare(xTruth, xOdom, u);

        % ------ Kalman Filter --------
        % Predict
        % ?
        xPred = doMotion(xEkf, u);
        jF = jacobF(xEkf, u);
        PPred = jF * PEkf * jF' + convQ;

        % Update
        % xEkf=?
        zPred = doObservation(z, xPred);
        jH = jacobH(xPred);
        y = z - zPred;
        S = jH * PPred * jH' + convR;
        K = PPred * jH' / S;
        xEkf = xPred + K * y;
        PEkf = (eye(3) - K * jH) * PPred;
        % -----------------------------

        % Simulation estimation
        estimation.time=[estimation.time; time];
        estimation.xTruth=[estimation.xTruth; xTruth'];
        estimation.xOdom=[estimation.xOdom; xOdom'];
        estimation.xEkf=[estimation.xEkf;xEkf'];
        estimation.GPS=[estimation.GPS; z'];
        estimation.u=[estimation.u; u'];

        % Plot in real time
        % Animation (remove some flames)
        if rem(i,removeStep)==0
            %hold off;
            plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
            plot(estimation.xOdom(:,1),estimation.xOdom(:,2),'.k', 'MarkerSize', 10);hold on;
            plot(estimation.xEkf(:,1),estimation.xEkf(:,2),'.r','MarkerSize', 10);hold on;
            plot(estimation.xTruth(:,1),estimation.xTruth(:,2),'.b', 'MarkerSize', 10);hold on;
            axis equal;
            grid on;
            drawnow;
            %movcount=movcount+1;
            %mov(movcount) = getframe(gcf);
        end 
    end
    close
    
    finalPlot(estimation);
 
end

% control
function u = robotControl(time)
    global endTime;

    T = 10; % sec
    Vx = 1.0; % m/s
    Vy = 0.2; % m/s
    yawrate = 5; % deg/s
    
    % half
    if time > (endTime/2)
        yawrate = -5;
    end
    
    u =[ Vx*(1-exp(-time/T)) Vy*(1-exp(-time/T)) degreeToRadian(yawrate)*(1-exp(-time/T))]';
    
end

% all observations for 
function [z, xTruth, xOdom, u] = prepare(xTruth, xOdom, u)
    global noiseQ;
    global noiseR;

    % Ground Truth
    xTruth=doMotion(xTruth, u);
    % add Motion Noises
    u=u+noiseQ*randn(3,1);
    % Odometry Only
    xOdom=doMotion(xOdom, u);
    % add Observation Noises
    z=xTruth+noiseR*randn(3,1);
end


% Motion Model
function x = doMotion(x, u)
    global dt;
    %?
    theta = x(3);
    vx = u(1);
    vy = u(2);
    w = u(3);
    if abs(w) < 1e-10
        x_next = x(1) + vx*cos(theta)*dt - vy*sin(theta)*dt;
        y_next = x(2) + vx*sin(theta)*dt + vy*cos(theta)*dt;
    else
        x_next = x(1) + (+vx*sin(theta + w*dt) - vy*cos(theta) + vy*cos(theta + w*dt) - vx*sin(theta))/w;
        y_next = x(2) + (-vx*cos(theta + w*dt) + vx*cos(theta) + vy*sin(theta + w*dt) - vy*sin(theta))/w;
    end
    theta_next = x(3) + w*dt;
    
    x = [x_next; y_next; theta_next];
end

% Jacobian of Motion Model
function jF = jacobF(x, u)
    global dt;
    %?
    theta = x(3);
    vx = u(1);
    vy = u(2);
    w = u(3);
    if abs(w) < 1e-10
        jF = [
            1, 0, (-vx * sin(theta) + vy * cos(theta)) * dt;
            0, 1, (vx * cos(theta) - vy * sin(theta)) * dt;
            0, 0, 1];
    else
        jF = [
            1, 0, (vx * (cos(theta+w*dt) - cos(theta)) + vy * (-sin(theta+w*dt) + sin(theta))) / w;
            0, 1, (vx * (sin(theta+w*dt) + sin(theta)) + vy * (cos(theta+w*dt) - cos(theta))) / w;
            0, 0, 1];
    end
end

%Observation Model
function x = doObservation(z, xPred)
    %?
    x = [xPred(1); xPred(2); xPred(3)];
 end

%Jacobian of Observation Model
function jH = jacobH(x)
    %?
    jH = eye(3);
end

% finally plot the results
function []=finalPlot(estimation)
    figure;
    
    plot(estimation.GPS(:,1),estimation.GPS(:,2),'*m', 'MarkerSize', 5);hold on;
    plot(estimation.xOdom(:,1), estimation.xOdom(:,2),'.k','MarkerSize', 10); hold on;
    plot(estimation.xEkf(:,1), estimation.xEkf(:,2),'.r','MarkerSize', 10); hold on;
    plot(estimation.xTruth(:,1), estimation.xTruth(:,2),'.b','MarkerSize', 10); hold on;
    legend('GPS Observations','Odometry Only','EKF Localization', 'Ground Truth');

    xlabel('X (meter)', 'fontsize', 12);
    ylabel('Y (meter)', 'fontsize', 12);
    grid on;
    axis equal;
    
    % calculate error
    % ?
    error_odom = sqrt((estimation.xTruth(:,1) - estimation.xOdom(:,1)).^2 + ...
                                  (estimation.xTruth(:,2) - estimation.xOdom(:,2)).^2);
    avg_error_odom = mean(error_odom);

    error_ekf = sqrt((estimation.xTruth(:,1) - estimation.xEkf(:,1)).^2 + ...
                              (estimation.xTruth(:,2) - estimation.xEkf(:,2)).^2);
    avg_error_ekf = mean(error_ekf);

    disp(['Odom Error: ', num2str(avg_error_odom)]);
    disp(['Ekf Error: ', num2str(avg_error_ekf)]);

end

function radian = degreeToRadian(degree)
    radian = degree/180*pi;
end