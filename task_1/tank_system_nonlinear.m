function dHdt = tank_system_nonlinear(t, H, a, b, A, V) %non-linear ODE
    %dHdt = -0.16*(a/A)*H + (b/A)*V;
    dHdt = (1/A)*(b*V-a*sqrt(H));
end