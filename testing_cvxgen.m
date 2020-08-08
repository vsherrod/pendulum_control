%parmaters
g = 9.81;
l1 = 1.0;
l2 = 1.0;
m1 = 1.0;
m2 = 1.0;

b1 = .7;
b2 = .7;

I1 = 1/12*m1*(5/4.0*l1^2+.1^2); %here is an issue?
I2 = I1;

%time step for discretization
timestep = .01;

%control weights
kp1 = 10.0; %100.0;
kp2 = 10.0;

params.Q1 = kp1;
params.Q2 = kp2;
params.Q3 = [0.0009 0;
            0 0.0001;];

params.R1 = 0.0;

params.R2 = 10.0;

%commanded states
params.q_goal = [pi/4.0; 0; -pi/4.0; 0;];

%starting torque (input) and states
params.tau_prev = [0.0; 0.0;];

q1 = 0.0;
q2 = 0.0;
q1_dot = 0.0;
q2_dot = 0.0;

x = [q1; q1_dot; q2; q2_dot;];

params.q_0 = x;

% A_c = [0 1 0 0;
%     -3*g/(2*l1)*cos(x(1)) - 3*m2/m1*g*(1/l1*cos(x(1)) + l1/(2*l1^2)*cos(x(1)+x(3))), -3*b1/(m1*l1^2), -3*m2*g*l2/(2*m1*l1^2)*cos(x(1)+x(3)), 0;
%     0 0 0 1;
%     -3*g/(2*l2)*cos(x(1)+x(3)), 0, -3*g/(2*l2)*cos(x(1)+x(3)), -3*b2/(m2*l2^2);];
% 
% B_c = [0 0;
%     3/(m1*l1^2) 0;
%     0 0;
%     0 3/(m2*l2^2);];

q_plot(1,:) = x';

steps = 1000;

for i=1:steps
    

    A_c = [0 1 0 0;
        -m1*g*l1/(2*I1)*cos(x(1)) - m2*g*l1/I1*cos(x(1)) - m2*g*l2/(2*I1)*cos(x(1)+x(3)), -b1/I1, -m2*g*l2/(2*I1)*cos(x(1)+x(3)), 0;
        0 0 0 1;
        -m2*g*l2/(2*I2)*cos(x(1)+x(3)), 0, -m2*g*l2/(2*I2)*cos(x(1)+x(3)), -b2/I2;];

    B_c = [0 0;
        1.0/I1 0;
        0 0;
        0 1.0/I2;];

    %These don't affect A and B in the discretization
    C_c = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1;];

    D_c = [0 0;
        0 0;
        0 0;
        0 0;];


    sys = ss(A_c, B_c, C_c, D_c);

    sysd = c2d(sys,timestep);

    params.A = sysd.A;
    params.B = sysd.B;

    [vars, status] = csolve(params);
    
    status

    tau = vars.u{1};

    tspan = [0 timestep];

    [t, x_ode] = ode45(@(t,x_ode) double_pendulum_system(t,x_ode,tau), tspan, params.q_0);

    params.q_0 = x_ode(end,:)';
    x = params.q_0

    params.tau_prev = tau;

    q_plot(i+1,:) = params.q_0';
    tau_plot(i,:) = tau';
end

t_plot = linspace(0,steps*timestep,length(q_plot));

figure(3)
for i=1:4
    subplot(2,2,i)
    plot(t_plot, q_plot(:,i), t_plot, params.q_goal(i)*ones(length(q_plot),i))
end


%plotting section for one pass through %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:1:21
    q(:,i) = vars.q{i};
    if i~=21
        tau(:,i) = vars.u{i};
    end
end

t_end = 20*timestep;
t = 0:timestep:t_end;

figure(1)
subplot(2,2,1)
plot(t, q(1,:))
title('q1')

subplot(2,2,3)
plot(t, q(2,:))
title('q1_dot')

subplot(2,2,2)
plot(t, q(3,:))
title('q2')

subplot(2,2,4)
plot(t, q(4,:))
title('q2_dot')

figure(2)
plot(t(1:20), tau(1,:), t(1:20), tau(2,:))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plotting at end of what behavior would be if it took all the commands
q_plot2(1,:) = params.q_0';
for i=1:length(tau(1,:))
     tspan = [0 timestep];

    [t, x_ode] = ode45(@(t,x_ode) double_pendulum_system(t,x_ode,tau(:,i)), tspan, params.q_0);
    
    params.q_0 = x_ode(end,:)';
    q_plot2(i+1,:) = params.q_0';
    
end
    
figure(4)
t_plot2 = 0:timestep:timestep*(length(q_plot2)-1);
for i=1:4
    subplot(2,2,i)
    plot(t_plot2, q_plot2(:,i), t_plot2, params.q_goal(i)*ones(length(q_plot2),i))
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Test objective function

score = 20*(params.Q1*(params.q_goal(1) - q_plot(end,1))^2 + params.Q2*(params.q_goal(3) - q_plot(end,3))^2) + params.R1*(params.q_goal(1) - q_plot(end,1))^2 + params.R2*(params.q_goal(3) - q_plot(end,3))^2

function output = double_pendulum_system(t,x,u)
    
    g = 9.81;
    l1 = 1.0;
    l2 = 1.0;
    m1 = 1.0;
    m2 = 1.0;

    b1 = .7;
    b2 = .7;
    
    I1 = 1/12*m1*(5/4.0*l1^2+.1^2);
    I2 = I1;
    
    output(1) = x(2);
    output(2) = u(1)/I1 - b1/I1*x(2) - m1*g*l1/(2*I1)*sin(x(1)) - m2/I1*g*(l1*sin(x(1))+l2/2.0*sin(x(1)+x(3)));
    output(3) = x(4);
    output(4) = u(2)/I2 - b2/I2*x(4) - m2*g*l2/(2*I2)*sin(x(1) + x(3));
    
    output = output';

end


