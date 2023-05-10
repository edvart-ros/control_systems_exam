function dHdt = tank_system_step_feedback(t, H, a, b, A, H_desired, H0, V) %linear ode
    dHdt = -(a/(2*A*sqrt(10)))*H + (b/A)*V;
end