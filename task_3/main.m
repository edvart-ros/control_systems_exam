XX = 24;
YY = 08;
ZZ = 18;
g = 9.81;
L_a = (10+XX*20/100)*0.01;
L_p = (5+YY*25/100)*0.01;
M_a = L_a/10+0.04;
M_p = ZZ/100;
J_a = (M_a *L_a^2)/12 ; J_p = (M_p*L_p^2)/12 ; J_t = J_a*J_p + J_a*M_p*(L_p/2)^2 + J_p*M_p* L_a^2;
A = [0 0 1 0 ; 0 0 0 1; 0 M_p^2*L_p^2*L_a*g/(4*J_t) 0 0;0 M_p*L_p*g*(J_a+M_p*L_a^2)/(2*J_t) 0 0];
B = [0;0;(J_p + 1/4*M_p*L_p^2)/J_t ; M_p*L_p*L_a/(2*J_t)];
C = [1 0 0 0; 0 1 0 0 ];
D = [0;0];
sys_ss = ss(A,B,C,D); %state space representation
H_matlab = tf(sys_ss);

%% From state space to transfer function
% INSERT DERIVATION OF GOING FROM SS TO TF
% I round off the entries in A to make my life a bit easier.
A = [0 0 1 0;
     0 0 0 1;
     0 181 0 0;
     0 782 0 0;];

B = [0;
     0;
     921;
     2921;];

C = [1 0 0 0; 
     0 1 0 0];

D = [0;
     0];

sys_ss = ss(A, B, C, D);
s = tf('s');
I = eye(4); % 4x4 identity
H_manual = C*((I*s-A)\B); %im usin the \ inverse operator since inv() gave me some extra poles at 0 for some reason

%% Verifying transfer functions
% The two transfer functions seemed a little different so i plotted their step responses to make sure they are really equivalent:
subplot(2,1,1);
step(H_manual(1), 'b-')
title('Theta part, manual')
subplot(2,1,2);
step(H_matlab(1), 'r-')
title('Theta part, tf()')
%%
close;
subplot(2,1,1);
step(H_manual(2), 'b-')
title('Alpha part, manual')
subplot(2,1,2);
step(H_matlab(2), 'r-')
title('Alpha part, tf()')
%%
% The manually computed and tf(sys_ss) transfer functions behave the same, so I conclude that they only look different due to 
% numerical inaccuracies.
%% Verifying simulink state space and transfer function representations
% I also checked if the state space and transfer function representations agreed in simulink:
% INSERT PICS HERE

%% Plant transfer functions
% Before I move on, i remove the extremely small coefficients in the transfer functions, as they have virtually no impact)
% I verified this by checking that the poles didnt change.
close;

H_theta = (921*s^2 - 191500)/(s^4 - 782*s^2); %theta transfer function
H_alpha  = (2921)/(s^2-782); %alpha transfer function
H = [H_theta;H_alpha];
%%
% We end up with the following plant transfer functions:
%%
% $$ H_{\theta} = \frac{921s^2-191500}{s^4-782s^2} $$
%%
% $$ H_{\alpha} = \frac{2921}{s^2-782} $$
%% Plant step response
close;
subplot(2,1,1);
step(H_theta, 'b')
title('Theta response')
subplot(2,1,2);
step(H_alpha, 'r')
title('Alpha response')
%% System stability and poles
% The system is clearly unstable in both $\theta$ and $\alpha$. 
%%
% Checking the poles with pzplot() and pole():
close;
pzplot(H);
pole(H_theta)
pole(H_alpha)

%%
% The transfer function $\frac{\theta (s)}{T(s)}$ has four poles:
%%
% $$[0, 0, 27.9643, -27.9643]$$
%%
% While the transfer function $\frac{\alpha (s)}{T(s)}$ has two:
%%
% $$[27.9643, -27.9643]$$
%%
close;

%% Feedback gain vector k
% We need to find a gain vector $\vec{k} = [k_1, k_2]^T$ Which brings the poles/eigenvalues to -10, for our system:
%%
% $$\dot{\vec{x}} = A\vec{x} + \vec{B}T$$
% First I find the values for $k_1$ and $k_2$ which bring the poles of $\theta$ and $\alpha$, respectively, to -10.

P = [-10, -10, -10, -10];
K = acker(A, B, P);
A_CL = A-B*K;
new_sys = ss(A_CL, B, C, D);
impulse(new_sys)




