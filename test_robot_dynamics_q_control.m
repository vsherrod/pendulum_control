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
timestep_control = 1/50.0;

%control weights
kp1 = 10.0; %100.0;
kp2 = 10.0;

kp = [kp1 0.0;
    0.0 kp2;];

kd = [5.0 0;
    0 5.0;];

params.Q1 = 0.0002;
params.Q2 = 0.0002;
params.Q3 = [1000 0;
            0 1000;];

params.R1 = 500.0;

params.R2 = 100.0;

%commanded states
params.q_goal = [pi/4.0; 0; -pi/4.0; 0;];

params.q_des_prev = [0.0; 0.0;];

q1 = 0.0;
q2 = 0.0;
q1_dot = 0.0;
q2_dot = 0.0;

x = [q1; q1_dot; q2; q2_dot;];

params.q_0 = x;

q_plot(1,:) = x';

control_steps = 1;

for i=1:control_steps
    
    C = [-0.5*m2*l1*l2*x(4)*sin(x(3)), -0.5*m2*l1*l2*cos(x(3))*(x(4)+x(2));
    0.5*m2*l1*l2*x(2)*sin(x(3)), 0;];

    Tau_grav = [-1.0*(m1*l1/2 + m2*l1)*g*sin(x(1)) - 0.5*m2*l2*g*sin(x(1) + x(3));
                -.5*m2*l2*g*sin(x(1)+x(3));];
            
    a = D_inv*(-C-b-kd);
    
    y = -D_inv*kp;
    
    A_c = [0 1 0 0;
           y(1,1) a(1,1) y(1,2) a(1,2);
           0 0 0 1;
           y(2,1) a(2,1) y(2,2) a(2,2);];

    B_c = [0 0;
           -y(1,1) -y(1,2);
           0 0;
           -y(2,1) -y(2,2);];
    

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

%     params.q_goal = [0.20319161029830202 0.8325913904724193 -0.8363810443482227 0.04331042079065206]';
%     
%     params.q_0 = [1.5717878173906188 1.5851723557337523 -1.497658758144655 -1.171028487447253]';
%     params.Q1 = 1.0514672033008299;
%     params.Q2 = 1.4408098436506365;
%     params.R1 = 1.0298762108785668;
%     params.R2 = 1.456833224394711;
%     params.A = [0.596576190459043, -0.8860508694080989, 0.705019607920525, 0.3634512696654033;
%                     -1.9040724704913385, 0.23541635196352795, -0.9629902123701384, -0.3395952119597214;
%                     -0.865899672914725, 0.7725516732519853, -0.23818512931704205, -1.372529046100147;
%                     0.17859607212737894, 1.1212590580454682, -0.774545870495281, -1.1121684642712744;]';
%     params.B = [-0.44811496977740495 1.7455345994417217;
%         1.9039816898917352 0.6895347036512547;
%         1.6113364341535923 1.383003485172717;
%         -0.48802383468444344 -1.631131964513103;];
% 
% 
%     params.B = [-0.44811496977740495 1.6113364341535923;
%         1.7455345994417217  1.383003485172717;
%         1.9039816898917352 -0.48802383468444344;
%         0.6895347036512547  -1.631131964513103;]
    
    [vars, status] = csolve(params);
    
    status
   
     tspan = linspace(0, timestep_control,2);
     
     q_des = vars.u{1};
     
     x_des = [q_des(1);
                0.0;
                q_des(2);
                0.0;];
     
%      tau = kp*(x_des - params.q0) - 
    
%     tau = [0;
%         0;];
    
%     tau = Tau_grav

    [t, x_ode] = ode45(@(t,x_ode) double_pendulum_system(t,x_ode,tau,D_inv,b), tspan, params.q_0);

    params.q_0 = x_ode(end,:)';
    x = params.q_0

    params.tau_prev = tau;
    
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
    
    g = 0.0;
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