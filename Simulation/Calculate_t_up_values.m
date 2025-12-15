clear;
clear hybrid_control_pendubot

q1_ref_last = 50*pi/180;

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

% plot(t_vals, q1_vals, 'LineWidth', 2); grid on

% swing-up target for link 1
%% What Q1s are possible
% lets use all Q1s every 10 degrees except for +-20 degrees close to
% horizontal so +- 20 degrres and +-160 to 200 or -160 to -200
% For a given angle we must find t_up Optimal
% To do this we run the simulation with a set of T_ups and plot results
%% Finding q1 reference trajectory
x0 = [-pi/2; 0 ;0;0];
q1_ref_initial = x0(1);



syms t tup q1_ref_final
q1_ref_traj = q1_ref_initial-(q1_ref_final-q1_ref_initial)*(-1/2+ 1/2*cos((t/tup) * pi));

dq1_ref_traj = diff(q1_ref_traj, t);
ddq1_ref_traj = diff(dq1_ref_traj, t);

q1_fun   = matlabFunction(q1_ref_traj,   'Vars', [t,tup,q1_ref_final]);
dq1_fun  = matlabFunction(dq1_ref_traj,  'Vars', [t,tup,q1_ref_final]);
ddq1_fun = matlabFunction(ddq1_ref_traj, 'Vars', [t,tup,q1_ref_final]);


kp = 65;   
kd = 3;

t_up_set =  0.2:.01:2.0;

num_tup = length(t_up_set);
figure;
hold on

number_of_Good =0;
Angle_Positive_setup=  20:10:160;
Radians_setup =Angle_Positive_setup*pi/180;
Angle_Negative_setup = -20:-10:-160;
Radians_setup =[  Radians_setup Angle_Negative_setup*pi/180];
num_Positions = length(Radians_setup);
best_Tups = zeros(2,num_Positions);


    best_Tup_q2 = 500;
    best_Tup_dq2 = 500;
    best_Tup_Tup = 0.0;
    for i = 1:num_tup
        tup_val = t_up_set(i);
        K =0;
        odefun = @(t,x) pendubot_dynamics(t, x, hybrid_control_pendubot(x,p,q1_ref_last,q1_fun(t,tup_val,q1_ref_last),dq1_fun(t,tup_val,q1_ref_last),ddq1_fun(t,tup_val,q1_ref_last),kp,kd,K,t,tup_val,2,false), p);
        clear hybrid_control_pendubot
        tspan = [0 10];
        [t,X] = ode45(odefun, tspan, x0);
        q1  = X(:,1);
        q2  = X(:,2);
        dq1 = X(:,3);
        dq2 = X(:,4);

        good_Tup = false;
    
        for k = 1:length(q1)
        
    
            % Condition: q2 close to 0 AND dq2 small
            q1_wrapped = mod(q1(k)+pi,2*pi)-pi;
            q2_wrapped = mod(q2(k) +pi,2*pi)-pi;
            near_zero_q2 = (abs(q1_ref_last+q2_wrapped -pi/2) < 0.7) ;
            slow_dq2     = abs(dq2(k)) < 1.0;
            q1_near_final = abs(q1(k) -q1_ref_last) < .4;
    
            if near_zero_q2 && slow_dq2 && q1_near_final
                good_Tup = true;
                if abs((q1_ref_last+q2_wrapped -pi/2)) < best_Tup_q2
                    best_Tup_q2 = q2(k);
                    best_Tup_dq2 = dq2(k);
                    best_Tup_Tup = tup_val;
                    q1_Best = q1;
                    q2_Best = q2 ;
                    dq1_Best =dq1;
                    dq2_Best = dq2;
                    t_Best = t;
                    time_Best = t(k);

                end
            end
    
        end
        
        if good_Tup
            number_of_Good =number_of_Good+1;
            subplot(3,1,1);
            plot(t(1:end), q1(1:end)); hold on;
            yline(q1_ref_last, '--');
            ylabel('q1 (rad)');
            title('Link 1 (actuated)');
            
            subplot(3,1,2);
            plot(t(1:end),mod(q2, -2*pi)); hold on;
            ylim([-2*pi 0])
            yline(pi/2 - q1_ref_last, '--');
            yline(pi/2 - q1_ref_last- 2*pi, '--');
            ylabel('q2 (rad)');
            xlabel('time (s)');
            title('Link 2 (unactuated)');
            subplot(3,1,3);
            plot(t(1:end), dq2 ); hold on;
            ylabel('dq2 (rad)');
            xlabel('time (s)');
        end
        

  
    

    


    end
   

disp(best_Tups)
disp('angle')
q2_wrapped =mod(best_Tup_q2,2*pi);

if abs(best_Tup_q2) > pi
    q2_Tup_Best = q2_wrapped;
else
    q2_Tup_Best =best_Tup_q2;
end

      

disp(q2_Tup_Best)

disp('velocity')
disp(best_Tup_dq2)
disp('Best Time')
disp(time_Best)
disp('Time Up')
disp(best_Tup_Tup)
figure;
subplot(3,1,1);
q1_ref = q1_ref_last;
        plot(t_Best(1:end), q1_Best); hold on;
        yline(q1_ref, '--');
        ylabel('q1 (rad)');
        title('Link 1 (unactuated)');

        subplot(3,1,2);
        plot(t_Best, q2_Best(1:end) ); hold on;
        ylabel('q2 (rad)');
        xlabel('time (s)');
        title('Link 2 (actuated)');
        subplot(3,1,3);
        plot(t_Best, dq2_Best ); hold on;
        ylabel('q2 (rad)');
        xlabel('time (s)');
        title('Link 2 (actuated)');

%% Finding q1 reference trajectory



q1_ref = q1_ref_last;
[A,B] = A_and_B(q1_ref_last);
% initial condition: both links hanging down, no motion

tspan = [0 10];

Q = [50 0 0 0; 
     0 30 0 0;
     0  0 5 0;
     0  0  0 5];
R = 1;

K = lqr(A,B,Q,R);

% Figure 3

% odefun = @(t,x) acrobot_dynamics(t, x, hybrid_control(x,p,q1_ref,kp,kd,K,t,SIGMA), p);
% [t3,X3] = ode45(odefun, tspan, x0);
% 
% % Figure 4
% kp = 20; 
% kd = 8;
% odefun = @(t,x) acrobot_dynamics(t, x, hybrid_control(x,p,q1_ref,kp,kd,K,t,SIGMA), p);
% [t4,X4] = ode45(odefun, tspan, x0);
% 
% % Figure 5
% kp = 20; 
% kd = 8;
% tspan = [0 2];
% odefun = @(t,x) acrobot_dynamics(t, x, hybrid_control(x,p,q1_ref,kp,kd,K,t,SIGMA), p);
% [t5,X5] = ode45(odefun, tspan, x0);
        q1_ref = q1_ref_last;
        tup_val = best_Tup_Tup;
        odefun = @(t,x) pendubot_dynamics(t, x, hybrid_control_pendubot(x,p,q1_ref,q1_fun(t,tup_val,q1_ref),dq1_fun(t,tup_val,q1_ref),ddq1_fun(t,tup_val,q1_ref),kp,kd,K,t,tup_val,2, false), p);
        clear hybrid_control_pendubot
        [t,X] = ode45(odefun, tspan, x0);
        
% Figure 6 -sgma2
% odefun = @(t,x) pendubot_dynamics(t, x, hybrid_control_pendubot(x,p,q1_ref,q1_fun(t),dq1_fun(t),ddq1_fun(t),kp,kd,K,t,tup_val,2), p);
% [t,X] = ode45(odefun, tspan, x0);

%sgma1
 % odefun = @(t,x) acrobot_dynamics(t, x, hybrid_control2(x,p,q1_ref,kp,kd,K,t,1), p);
 % [t7,X7] = ode45(odefun, tspan, x0);
% q17  = X7(:,1);
% q27  = X7(:,2);
% dq17 = X7(:,3);
% dq27 = X7(:,4);


q1  = X(:,1);
q2  = X(:,2);
dq1 = X(:,3);
dq2 = X(:,4);

% figure;
% subplot(2,1,1);
% plot(t7, q17); hold on;
% yline(q1_ref, '--');
% ylabel('q1 (rad)');
% title('Link 1 (unactuated)');
% 
% subplot(2,1,2);
% plot(t7, q27 );
% ylabel('q2 (rad)');
% xlabel('time (s)');
% title('Link 2 (actuated)');
% figure;
% T_CHECK = 600;
figure;
subplot(2,1,1);
plot(t(1:end), q1(1:end)); hold on;
yline(q1_ref, '--');
ylabel('q1 (rad)');
title('Link 1 (unactuated)');

subplot(2,1,2);
plot(t(1:end), mod(q2(1:end)+pi,2*pi )-pi );
yline(pi/2 -q1_ref, '--');
ylabel('q2 (rad)');
xlabel('time (s)');
title('Link 2 (actuated)');

% Plot the joint velocities
figure;
plot(t(1:end), dq1(1:end), t(1:end), dq2(1:end));
ylabel('Joint Velocities (rad/s)');
xlabel('time (s)');
title('Joint Velocities of the Acrobot');
legend('dq1', 'dq2');

