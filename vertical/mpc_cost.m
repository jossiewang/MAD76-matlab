function J = mpc_cost(u, x0,N,spline,dt)
% Initialize state and cost
x=x0; 
x_last=x0;
u_last=u(1:2);
J=0;
dist=0;

% Loop over prediction horizon
Q=[54.6098,0,0;0,0,0;0,0, 30.1091];
R=[0.1000,0;0,5];
% Best params:  [54.6098, 41.8648, 20.1091]
% Best params:  [0.1000, 52.7927, 100.0000]
% Q_der=[0.2,0,0,0,0,0,0
%         0,0.1,0,0,0,0,0
%         0,0, 0.05,0,0,0,0
%         0,0,0,0,0,0,0;
%         0,0,0,0,0.1,0,0;
%         0,0,0,0,0,0,0;
%         0,0,0,0,0,0,0;];
% R_der=[0.1000,0;0,0];
% 
% Q_lin=[54.6098,41.8648,10.1091];
% R_lin=[0.1000,0];
for i=1:N-1
    
    %u_new=u(:,i);
    %chatgpt adaptation for col vec
    idx = 2*i-1;
    u_new = u(idx:idx+1);
    %
    
    [x_diff,dist] = error_dynamics(spline,x);
    %e_lane=x_diff(1);

    J=double(J+(x_diff.'*Q*x_diff+...
        u_new.'*R*u_new-0.5'*dist));
    
    % if i>=2
    %     x_der=x-x_last;
    %     u_der=u_new-u_last;
    %     J=double(J+x_der.'*Q_der*x_der+u_der.'*R_der*u_der);
    %     u_last=u_new;
    % 
    % end
    x_last=x;
    x=simulate_dynamics_rk4(x, u_new, dt);
    
end


% Terminal cost
% P=100.0000;
% x_fin=1.84;
% J=double(J+P*((dist-x_fin)/x_fin).^2);
end