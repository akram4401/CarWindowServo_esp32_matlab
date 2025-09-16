% esp32_live_plot.m
% Step response test with ESP32 (270° motor servo + PID + Deadband)

clear; clc; close all;

% ==== Serial Setup ====
port = 'COM9';   % <-- CHANGE if needed
baud = 115200;
s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

% ==== PID + Deadband Settings ====
Kp = 1.9480;  %Kp = 1.9667; 
Ki = 4.4022;  %6.0647
Kd = 0.2155;  %0.1594
DB = 2;   % deadband tolerance in degrees

% Send settings to ESP32
writeline(s, sprintf("KP %.3f", Kp));
writeline(s, sprintf("KI %.3f", Ki));
writeline(s, sprintf("KD %.3f", Kd));
writeline(s, sprintf("DB %d", DB));
pause(0.5);

% ==== Step Sequence ====
setpoints = [45, 100, 90, 180, 0]; 
stepDelay = 5;  % seconds to hold each setpoint

% ==== Init Live Plot ====
figure('Name','ESP32 Live Step Response','NumberTitle','off');
hPos = animatedline('LineWidth',1.6,'Color','b');   % motor position
hSP  = animatedline('LineWidth',1.2,'LineStyle','--','Color','r'); % setpoint
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('MotorPos','Setpoint','Location','best');
grid on;

% ==== Data Logging ====
logTime = [];
logSP   = [];
logPos  = [];
logErr  = [];
logOut  = [];

startTime = tic;
disp("=== Starting Step Response Test ===");

for sp = setpoints
    writeline(s, sprintf("SP %d", sp));
    fprintf("Setpoint -> %d deg\n", sp);

    tStepStart = toc(startTime);
    while toc(startTime) - tStepStart < stepDelay
        if s.NumBytesAvailable > 0
            line = readline(s);
            line = strtrim(line);
            % DEBUG: print raw lines
            % disp("RAW: " + line);

            if startsWith(line,"T,")
                parts = split(line,",");
                if numel(parts) >= 10
                    % Parse as floats (not ints!)
                    tms   = str2double(parts{2})/1000; % ms -> s
                    spVal = str2double(parts{4});
                    posVal= str2double(parts{6});
                    errVal= str2double(parts{8});
                    outVal= str2double(parts{10});

                    if ~isnan(tms) && ~isnan(posVal)
                        % Append to log
                        logTime(end+1) = tms;
                        logSP(end+1)   = spVal;
                        logPos(end+1)  = posVal;
                        logErr(end+1)  = errVal;
                        logOut(end+1)  = outVal;

                        % Update live plot
                        addpoints(hPos, tms, posVal);
                        addpoints(hSP,  tms, spVal);
                        xlim([max(0,tms-20), tms+1]); 
                        drawnow limitrate;
                    end
                end
            end
        else
            pause(0.01);
        end
    end
end

disp("=== Step Response Finished ===");

% ==== Save CSV ====
if ~isempty(logTime)
    T = table(logTime', logSP', logPos', logErr', logOut',...
        'VariableNames',{'Time_s','Setpoint','Position','Error','Output'});
    writetable(T,'step_response_log.csv');
    disp("Saved -> step_response_log.csv");
else
    warning("⚠ No telemetry logged! Check ESP32 serial output format.");
end

% ==== Final Plots ====
if ~isempty(logTime)
    figure('Name','Step Response Analysis','NumberTitle','off');

    subplot(3,1,1);
    plot(logTime, logSP,'--r','LineWidth',1.2); hold on;
    plot(logTime, logPos,'b','LineWidth',1.5);
    ylabel('Angle (deg)');
    legend('Setpoint','Motor Position');
    title(sprintf('Step Response (DB = ±%d°)', DB));
    grid on;

    subplot(3,1,2);
    plot(logTime, logErr,'m','LineWidth',1.2);
    ylabel('Error (deg)');
    title('Error vs Time');
    grid on;

    subplot(3,1,3);
    plot(logTime, logOut,'k','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Control Out');
    title('PID Output');
    grid on;
end
