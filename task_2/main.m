%% Task 2
% Analysis of piloted airplaine stability.

%% Transfer function
% the transfer function, with canard deflection as input and pitch altitude as output, is given as:
%%
% $$ \frac{\theta}{\delta_c} = \frac{s+24}{(s-8)(s-18)} $$
%
% This system is clearly unstable, as both poles are positive real numbers (8 and 18).
% We can verify this by using pzplot and looking at the poles:

s = tf('s');
sys = (s+24)/((s-8)*(s-18));
pzplot(sys);


%%
% As we can see, both poles have positive real part, so the system must be unstable

%% Step response and stability
close;
step(sys);

%% 
% The output of our system blows up to infinity, so it is clearly unstable.  
%
% The open-loop system does not satisfy BIBO, and requires closed loop control to become stable.

%% Propotional control transfer function
% We add a proportional control and find an equivalent transfer function for the whole system.
% I use the simulink block diagram to help find the expression for the new system.
%%
% $$ \theta = K_p \theta_c H(s) - K_p\theta H(s)$$
%%
% $$\theta(K_p H(s) + 1) = \theta_c K_p H(s)$$
%%
% $$\frac{\theta}{\theta_c} = \frac{K_p H(s)}{K_p H(s) + 1}$$
%%
% I insert the plant model $H(s)$ to obtain the full closed loop transfer function:
%%
% $$\frac{\theta}{\theta_c} = \frac{K_p \frac{s+24}{s^2-26+144}}{K_p \frac{s+24}{s^2-26+144} + 1} $$
%%
% Finally, simplifying the fraction gives:
%%
% $$\frac{\theta}{\theta_c} = \frac{K_p s+ K_p 24}{s^2+(K_p-26)s+(144+24K_p)} $$

%%
% Now that we have obtained the transfer function for our controlled system,
% We can plot the step responses for varying Kp's (proportional gains):
figure;
hold on;
for Kp = [50 110 170 200]
    closed_loop = (Kp*s+Kp*24)/(s^2+(Kp-26)*s+(144+24*Kp));
    step(closed_loop);
end

title('Step Response with Different Kp');
legend('Kp = 50', 'Kp = 110', 'Kp = 170', 'Kp = 200');
%%
% Increasing the proportional gain results in a shorter rise time, and a smaller steady state error. 
% However, we cannot completely eliminate the steady state error completely, without introducing an integral term.
%% P-control stability
% We can also plot the poles of this system as a function of the proportional gain $K_p$
% The poles are given by the roots of the polynomial in the denominator of our transfer function:
% We can use the quadratic formula to find the roots and plot $Re\{s\}$ and $Im\{s\}$ separately.
% We solve the equation:
%%
% $$ s^2+(Kp-26)s+(144+24)Kp = 0 $$
%
close;

Kp = -50:0.05:40; %range of Kp's to analyse
%find poles using the quadratic formula
s_1 = (-(Kp-26)+sqrt((Kp-26).^2-4*(144+24.*Kp)))/2; 
s_2 = (-(Kp-26)-sqrt((Kp-26).^2-4*(144+24.*Kp)))/2;

figure;

subplot(2,1,1);
grid on;
hold on;
plot(Kp, real(s_1), 'DisplayName', 'Root 1', 'LineStyle', '-');
plot(Kp, real(s_2), 'DisplayName', 'Root 2', 'LineStyle', '--');
plot(Kp, zeros(size(Kp)), 'k', 'LineWidth', 1);  % x-axis
xlabel('Kp'); ylabel('Real Part of Roots');
title('Real part of poles');
legend;

subplot(2,1,2);
grid on;
hold on;
plot(Kp, imag(s_1), 'DisplayName', 'Root 1', 'LineStyle', '-');
plot(Kp, imag(s_2), 'DisplayName', 'Root 2', 'LineStyle', '--');
plot(Kp, zeros(size(Kp)), 'k', 'LineWidth', 1);  % x-axis
xlabel('Kp'); ylabel('Imaginary Part of Roots');
title('Imaginary part of poles');
legend;




%% Kp-stable values
% From inspecting the plot we see that both poles have negative real part when $K_p > 26$.
%
% For the gain, i somewhat arbitrarily chose $Kp = 147.3$, which is where the poles start to separate again:
close;
figure;
grid on;
hold on;

Kp = 144:0.05:150;
s_1 = (-(Kp-26)+sqrt((Kp-26).^2-4*(144+24.*Kp)))/2;
s_2 = (-(Kp-26)-sqrt((Kp-26).^2-4*(144+24.*Kp)))/2;
plot(Kp, real(s_1), 'DisplayName', 'Root 1', 'LineStyle', '-');
plot(Kp, real(s_2), 'DisplayName', 'Root 2', 'LineStyle', '--');

Kp = 147.3;
plot(Kp, (-(Kp-26)+sqrt((Kp-26).^2-4*(144+24.*Kp)))/2, 'marker', '^', 'Color', 'k')
xlabel('Kp'); ylabel('Real Part of Roots');
title('Optimal poles');
legend;

%%
% Verifying that Kp = 26 gives us zero-poles
close;
Kp = 26;
closed_loop = (Kp*s+Kp*24)/(s^2+(Kp-26)*s+(144+24*Kp));
real(pole(closed_loop))

%%
% We can check the poles for our new controller with $Kp = 147.3$
Kp = 147;
closed_loop = (Kp*s+Kp*24)/(s^2+(Kp-26)*s+(144+24*Kp));
real(pole(closed_loop))
%%
close;
pzplot(closed_loop);

%% Control system step response
% We have verified that the poles of our new system are all negative real part. This means the system stable, and we can verify this
% by viewing the step response of the closed loop system:

step(closed_loop)
[y,t]=step(closed_loop); %save the output values to check steady state
SS_error = abs(1-y(end));


%%
% Thus we have verified that our control system is stable and reaches a steady state error of 3.89%, with a steady state value of
%%
% $$(1-\frac[3.89}{100} = 0.9611 $$
%%
% We do have some overshoot here,
% which could be mitigated by incorporating a derivative term (PD control)
% We can also check that our simulink model agrees with our matlab simulations.

%% Simulink step response
% Both the original simulink block diagram and the reduced transfer function show the same step response as the matlab code:
%
% <<final_step_response.png>>
%

