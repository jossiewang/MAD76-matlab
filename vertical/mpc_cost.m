function J = mpc_cost(u, x0,N,spline,dt)
% Initialize state and cost
x=x0; 
J=0;
dist=0;

% Loop over prediction horizon
Q=[5,0,0;0,5,0;0,0, 10];
R=[0,0;0,0];

for i=1:N-1
    
    %u_new=u(:,i);
    %chatgpt adaptation for col vec
    idx = 2*i-1;
    u_new = u(idx:idx+1);
    %
    
    [x_diff,dist] = error_dynamics(spline,x,u_new,dt);
    J=double(J+(x_diff.'*Q*x_diff+u_new.'*R*u_new));
    
    x=simulate_dynamics_rk4(x, u_new, dt);
    
end


% Terminal cost
P=4;
x_fin=1.84;
J=double(J+P*((dist-x_fin)/x_fin).^2);
end