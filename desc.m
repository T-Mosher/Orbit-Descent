% yet another attempt to compute the best powered descent trajectory
clear all
close all

% directionality -  1 -> down range, 2 -> vertical

% KSP "Spark" engine parameters
% Isp: 300 (vac)
% thrust: 18.0 kN (vac)
% fuel consumption: 1.22 units/s
% mass: 0.10 t  (100 kg)
% mass flow rate = thrust / (Isp * g)= 60.0 = 18000 / (300 * 9.81) = 6.11 kg/sec
% 6.11 kg/sec @ 5 kg/fuel unit = 1.22 fuel units/sec

% T100 fuel tank: 100 units capacity
% fuel mass = 100 units * 5 kg/unit = 500 kg


Tmax = 500;     % maximum run time
v = [550 ; 0];   % initial velocity - [horz, vert]
%v = [0 ; -100]; % vertical descent
Ft = 18000;       % Thrust in newtons, "Spark" engine
T = 0.05;           % initial throttle is %% (AP control only)
%Ft = 0;          % No thrust -> freefall
M0 = 600;        % dry mass in kg
Mf = 500;        % fuel mass in kg (K100 tank)
s = [0 ; 12000]; % initial position, s(2) is height
t = 0;           % initial time in seconds
dt = 0.1;        % time increment
%Km = 0.0005;        % mass rate at full throttle
Km = 6.11;        % mass rate at full throttle, kg/sec
ag = [0 ; -1.63];   % acceleration due to gravity m/sec^2
%ag = [0 ; 0];      % disable gravity for testing
Kap = 550/12000;      % average vertical rate (550 m/s over 12000 meters)
Kfinal = 100;       % height of vertical descent mode, in meters
Ktd = 5;        % target vertical touchdown rate in m/s
v_err_old = 0;    % initial value for velocity error
Kp = 0.05;       % throttle proportional constant
Kd = 0.3;       % throttle derivative constant
Told = 0.1;       % initial throttle value - can't be zero
T_FILTER = [.2 .2 .2 .2 .2]; % Coefficients for 5-unit low-pass filter
Tvec = zeros(5,1);    % data vector for raw Throttle command filter

dl = [];
while (s(2) > 0)
%for i=1:10
  %theta = atan(-v(1) / v(2))
  theta = atan2(v(1), -v(2));
  
  % various autopilot stuff here
  if 0
    % simple fixed throttle
    f = [-Ft*sin(theta); Ft*cos(theta)];
  end
  
  vt = sqrt(sumsq(v));  % total velocity
  
  if 0
    % simple AP
    if (s(2) < Kfinal)
      % we're in final vertical approach
      if (vt < Ktd)
         T = T * .99;   % reduce throttle by 1 pct
         disp("vert descent: reducing throttle");
         %pause
      end
    else
      % we're in descent mode
      if (s(2) * Kap) > vt
        T = T * .99;   % reduce throttle by 1 pct
        disp("descent mode: reducing throttle");
        %pause
      end
    end    
     
  end
  
% compute the target velocity for the descent profile
if 0
  % linear profile
  v_target = s(2) * Kap;
end
if 1
  % parabolic profile
  v_target = 109 * sqrt(s(2)) * Kap;   % some sort of quadratic profile
end
  
if 1
  % auto-throttle based on Surveyor method
  % Proportional & Derivative controller based on velocity error vs. altitude
  if (s(2) < Kfinal)
    v_target = Ktd;          % constant descent rate below Kfinal
  end
  
  % compute the error term
  v_err = vt - v_target;
  v_err_dot = v_err - v_err_old;
  v_err_old = v_err;
  
  % compute PD correction
  Kcorr = Kp*v_err + Kd*v_err_dot;
  
  if 0
    % unfiltered throttle
    % note: this only works because T never gets completely to zero
    % don't know why it jumps from full to zero so quickly
    %T = T * (1 + Kcorr);    % change the throttle setting proportionally
  end
  
  if 0
    % filtered throttle - shockingly bad
    %Tnew = Told * (1 + Kcorr);    % change the throttle setting proportionally
    %T = (0.9 * Told) + (0.1 * Tnew);
    %Told = Tnew;  
  end
  
  if 0
    % throttle bumper - very unstable
    if (Kcorr > 0)
      T = T + 0.01;   % increase throttle 
    else
      T = T - 0.01;   % decrease throttle
    end
  end
  
  if 1
    % proportional throttle delta
    T = T + Kcorr;
    
    % Does This Do Any Good? throttle low-pass filter
    Tvec(2:5) = Tvec(1:4);    % rotate in the new sample
    Tvec(1) = T;
    T_cmd = T_FILTER * Tvec;
    T = T_cmd;        % activates the filter
  end
  
  % physical throttle limits 
  if T > 1.0
    T = 1.0;
  end
  if T < 0
    T = 0.0;
  end
   
end
  
  % compute the force due to the commanded throttle setting
  f = T*[-Ft*sin(theta); Ft*cos(theta)]; 
  
  % vehicle dynamics here
  if (Mf < 0)
    % out of fuel, so no thrust
    disp("No fuel")
    a = 0;
  else
    a = f / (M0 + Mf);       % M0 + m_fuel
  end
  % apply gravity
  a_new = a + ag;
  v_delta = a_new * dt;
  v_new = v + v_delta;
  s_delta = v_new * dt;
  s = s + s_delta;
  % increments for the next loop
  a = a_new;
  v = v_new;
  t = t + dt;
  Mf = Mf - (T*Km*dt);
  % data logging
  dl = [dl ; theta*180/pi a(:)' v(:)' s(:)' Mf T vt Kcorr];
  if t > Tmax
    disp("Max. time expired...")
    break 
  end
end
% summarize the run
plot(dl(:,6), dl(:,7),'o-')
title("height vs. range")
figure
plot(dl(:,6), dl(:,1),'o-');
title("theta vs. range")
figure
plot(dl(:,6), dl(:,4), 'o-', dl(:,6), dl(:,5), 'x-');
title("velocities vs. range")

subplot(3,1,1)
plot(dl(:,4),'o-')
title("velocity x vs. time")
subplot(3,1,2)
plot(dl(:,5),'o-')
title("velocity y vs. time")
subplot(3,1,3)
plot(dl(:,1),'o-')
title("theta vs time")

figure
plot(dl(:,10),'o-')
title("total velocity vs time")
figure
plot(dl(:,7),'o-')
title("height vs. time")

% plot the autopilot performance
figure
plot(dl(:,7),dl(:,10),'o')
hold
%plot([0 Kfinal dl(1,7)],[Ktd Ktd dl(1,10)],'r-')
% plot the linear auto pilot line
plot([0 Kfinal 12000],[Ktd Ktd 550],'r-')
title("AP Perf: total v vs. height")
hold off
% note: to plot the generic autopilot line, need a function that returns it
% and to plot over a vector of range values.

figure
plot(dl(:,9),'xr-')
title("throttle vs time")

figure
plot(dl(:,11),'xr-')
title("Kcorr vs time")

disp("Final velocity:")
v
disp("Fuel remaining:")
Mf
disp("time:")
t