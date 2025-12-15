
clear;
clear hybrid_control_pendubot
%% FIX EoM, Fix Zero Dynamics, Comment PFL and LQR setup, Comment A and B, Comment Each Figure, Fix Hybrid_Control

%% HELPERS: EOM, ZeroDynamics, A_and_B, acrobot_dynamics, hybrid_control

%% EoM derivation

% Spong_EoMs = EOM();
% fprintf('\n');
% fprintf('Equation 1 = \n');
% disp(Spong_EoMs(1))
% fprintf('\n');
% fprintf('Equation 2 = \n');
% disp(Spong_EoMs(2))

%% Graph of zero dynamics

%y =ZeroDynamics();

%% PFL and LQR response

% parameters
p.m1 = 1.0969;     % Mass of link 1 (kg)
p.m2 = 0.8356;     % Mass of link 2 (kg)
p.l1 = 0.1524;     % Length of link 1 (m)
p.l2 = 0.2286;     % Length of link 2 (m)
p.lc1 = 0.0754;    % Distance to center of mass of link 1 (m)
p.lc2 = 0.0754;    % Distance toa center of mass of link 2 (m)
p.I1 = 0.0056;   % Moment of inertia of link 1 (kg·m^2)
p.I2 = 0.0059;   % Moment of inertia of link 2 (kg·m^2)
p.g = 9.81;     % Gravity (m/s^2)

SIGMA =1;
tspan = [0 10];
 



%q1 = cos^(-1)(t/((m1*lc1 +m1l2)*g)) = cos^(-1)(t/(PHI4*g)

% So if we determine equilibria in terms of q1 position and we want to 
% Go from equilibria X to equilibria Y 
% bottom-> swing to q1X -> balance ->drop-> swing to Q1Y ->balance
% 

 % (q1,q2,dq1,dq2)
% t_vals = linspace(0, tup, 500);
% q1_vals =q1_ref_initial-(q1_ref_final-q1_ref_initial)*(-1/2+ 1/2*cos((t_vals/tup) * pi));
% figure;
% plot(t_vals, q1_vals, 'LineWidth', 2); grid on

% swing-up target for link 1





% for i = 1:num_kd
%         kd_val = kd_set(i);
% 
%         odefun = @(t,x) pendubot_dynamics(t, x, hybrid_control_pendubot(x,p,q1_ref,q1_fun(t,tup_val),dq1_fun(t,tup_val),ddq1_fun(t,tup_val),kp,kd_val,K,t,tup_val,1, false), p);
%         [t,X] = ode45(odefun, tspan, x0);
% 
%         q1  = X(:,1);
%         q2  = X(:,2);
%         dq1 = X(:,3);
%         dq2 = X(:,4);
% 
%         good_Tup = false;
% 
%         for k = 1:length(q1)
% 
% 
%             % Condition: q2 close to 0 AND dq2 small
%             q1_wrapped = mod(q1(k)+pi,2*pi)-pi;
%             q2_wrapped = mod(q2(k) +pi,2*pi)-pi;
%             near_zero_q2 = (abs((q1_wrapped+q2_wrapped -pi/2)) < 0.4) ;
%             slow_dq2     = abs(dq2(k)) < 0.2;
%             q1_near_final = abs(q1_wrapped  -q1_ref_last) < .2;
% 
%             if near_zero_q2 && slow_dq2 && q1_near_final
%                 good_Tup = true;
%                 if abs((q1_wrapped+q2_wrapped -pi/2)) < best_Tup_q2
%                     best_Tup_q2 = q1_wrapped+q2_wrapped-pi/2;
%                     best_Tup_dq2 = dq2(k);
%                     best_Tup_Tup = tup_val;
%                     q1_Best = q1;
%                     q2_Best =  mod(q2 +pi,2*pi)-pi; 
%                     dq1_Best =dq1;
%                     dq2_Best = dq2;
%                     t_Best = t;
%                     time_Best = t(k);
%                     kd_Best = kd_val;
% 
%                 end
%             end
% 
%         end
%         if good_Tup
%             number_of_Good =number_of_Good+1;
%             subplot(3,1,1);
%             plot(t(1:end), q1(1:end)); hold on;
%             yline(q1_ref_last, '--');
%             ylabel('q1 (rad)');
%             title('Link 1 (unactuated)');
% 
%             subplot(3,1,2);
%             plot(t(1:end), mod(q2(1:end)+pi,2*pi)-pi ); hold on;
%             yline(pi/2-q1_ref_last, '--',pi/2-q1_ref_last);
%             ylabel('q2 (rad)');
%             xlabel('time (s)');
%             title('Link 2 (actuated)');
%             subplot(3,1,3);
%             plot(t(1:end), dq2 ); hold on;
%             ylabel('q2 (rad)');
%             xlabel('time (s)');
%             title('Link 2 (actuated)');
%         end
% end
% 
% disp(kd_Best)
% disp('time of Best')
% disp(time_Best )
% disp('BEST')
% disp(best_Tup_q2)
% 
% figure;
% T_CHECK = 600;
% subplot(2,1,1);
% plot(t_Best(1:end), q1_Best(1:end)); hold on;
% yline(q1_ref, '--', q1_ref);
% ylabel('q1 (rad)');
% title('Link 1 (unactuated)');
% 
% 
% q2_absolute = q2_Best;
% subplot(2,1,2);
% plot(t_Best(1:end), mod(q2_absolute(1:end)+pi,2*pi)-pi );
% yline(pi/2 - q1_ref, '--',pi/2 - q1_ref);
% ylabel('q2 (rad)');
% xlabel('time (s)');
% title('Link 2 (actuated)');
% 
% % Plot the joint velocities
% figure;
% plot(t(1:end), dq1(1:end), t(1:end), dq2(1:end));
% ylabel('Joint Velocities (rad/s)');
% xlabel('time (s)');
% title('Joint Velocities of the Acrobot');
% legend('dq1', 'dq2');
% K = lqr(A,B,Q,R);
% clear hybrid_control_pendubot
% disp('STARTING LQR')
angle = 40*pi/180;
x0 = [angle; pi/2-angle;0;0];
q1_ref = angle;
[A,B] = A_and_B_pendubot(q1_ref);
% initial condition: both links hanging down, no motion



Q = [200^2 0 0 0; 
     0 4^2 0 0;
     0  0 2^2 0;
     0  0  0 2^2];


R = 1;

K = lqr(A,B,Q,R);
odefun = @(t,x) pendubot_dynamics(t, x, hybrid_control_pendubot_LQR(x,p,q1_ref,K), p);
[t,X] = ode45(odefun, tspan, x0);
        q1  = X(:,1);
        q2  = X(:,2);
        dq1 = X(:,3);
        dq2 = X(:,4);

figure;
subplot(2,1,1);
plot(t, q1); hold on;
yline(q1_ref, '--', q1_ref);
ylabel('q1 (rad)');
title('Link 1 (actuated)');


q2_absolute = q2;
subplot(2,1,2);
plot(t(1:end), mod(q2_absolute(1:end)+pi,2*pi)-pi );
yline(pi/2 - q1_ref, '--',pi/2 - q1_ref);
ylabel('q2 (rad)');
xlabel('time (s)');
title('Link 2 (unactuated)');

figure;
plot(t(1:end), dq1(1:end), t(1:end), dq2(1:end));
ylabel('Joint Velocities (rad/s)');
xlabel('time (s)');
title('Joint Velocities of the Acrobot');
legend('dq1', 'dq2');
K = lqr(A,B,Q,R);
clear hybrid_control_pendubot