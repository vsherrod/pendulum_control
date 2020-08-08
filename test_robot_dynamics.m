%parameters
g = 9.81;
l1 = 1.0;
l2 = 1.0;
m1 = 1.0;
m2 = 1.0;

b1 = .7;
b2 = .7;

b = [b1 0;
    0 b2;];

I1 = 1/12*m1*(l1^2+.1^2); 
I2 = I1;

D = [I1+I2, I2;
    I2 I2;];

D_inv = inv(D);

%time step for control discretization
timestep_control = .001;

%control weights
kp1 = 10.0; %100.0;
kp2 = 10.0;

params.Q1 = kp1;
params.Q2 = kp2;
params.Q3 = [0.0 0;
            0 0.0;];

params.R1 = 0.0;

params.R2 = 0.0;

%commanded states
params.q_goal = [pi/2.0; 0; pi/4.0; 0;];

%starting torque (input) and states
params.tau_prev = [0.0; 0.0;];

q1 = 0.0;
q2 = 0.0;
q1_dot = 0.0;
q2_dot = 0.0;

x = [q1; q1_dot; q2; q2_dot;];

params.q_0 = x;

q_plot(1,:) = x';

control_steps = 1000;

for i=1:control_steps
    
    C = [-0.5*m2*l1*l2*x(4)*sin(x(3)), -0.5*m2*l1*l2*cos(x(3))*(x(4)+x(2));
    0.5*m2*l1*l2*x(2)*sin(x(3)), 0;];

    Tau_grav = [-1.0*(m1*l1/2 + m2*l1)*g*sin(x(1)) - 0.5*m2*l2*g*sin(x(1) + x(3));
                -.5*m2*l2*g*sin(x(1)+x(3));];
            
    a = D_inv*(-C-b);
    
    A_c = [0 1 0 0;
           0 a(1,1) 0 a(1,2);
           0 0 0 1;
           0 a(2,1) 0 a(2,2);];

    B_c = [0 0;
           D_inv(1,1) D_inv(1,2);
           0 0;
           D_inv(2,1) D_inv(2,2);];
    

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

    sysd = c2d(sys,timestep_control);

    params.A = sysd.A;
    params.B = sysd.B;

    %implicit differentiation
%     params.A = inv(eye(4)-timestep_control*A_c);
%     
%     params.B = timestep_control*B_c;
    
%     %explicit differentiation
%     params.A = timestep_control*A_c+eye(4);
%     
%     params.B = timestep_control*B_c;

    [vars, status] = csolve(params);
    
    status;

    tau = vars.u{1} - Tau_grav;

%     tspan = [0 timestep];
    
    tspan = linspace(0, timestep_control,2);
    
%     tau = [0;
%         0;];
    
%     tau = Tau_grav

    [t, x_ode] = ode45(@(t,x_ode) double_pendulum_system(t,x_ode,tau,D_inv,b), tspan, params.q_0);

    params.q_0 = x_ode(end,:)';
    x = params.q_0

    params.tau_prev = tau;

    q_plot(i+1,:) = params.q_0';
    tau_plot(i,:) = tau';
end

%% Plotting Section
t_plot = linspace(0,control_steps*timestep_control,length(q_plot));

figure(1)
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

t_end = 20*timestep_control;
t = 0:timestep_control:t_end;

figure(2)
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

figure(3)
plot(t(1:20), tau(1,:), t(1:20), tau(2,:))



function output = double_pendulum_system(t,x,u,D_inv,b)
    
    g = 9.81;
    l1 = 1.0;
    l2 = 1.0;
    m1 = 1.0;
    m2 = 1.0;

    C = [-0.5*m2*l1*l2*x(4)*sin(x(3)), -0.5*m2*l1*l2*cos(x(3))*(x(4)+x(2));
    0.5*m2*l1*l2*x(2)*sin(x(3)), 0;];

    Tau_grav = [-1.0*(m1*l1/2 + m2*l1)*g*sin(x(1)) - 0.5*m2*l2*g*sin(x(1) + x(3));
                -.5*m2*l2*g*sin(x(1)+x(3));];
            
    q_dd = D_inv*((-C-b)*[x(2); x(4);] + u + Tau_grav);
    
    output(1) = x(2);
    output(2) = q_dd(1);
    output(3) = x(4);
    output(4) = q_dd(2);
    
    output = output';

end



