function draw_pendulum_2x2(y, plot_number)
    theta = y(1);
    alpha = y(2);
    L1 = 1;
    L2 = 1;
    
    % have to draw to bars that are linked:
    % bar 1 will start at [0, 0] with length L1, angle theta
    % bar 2 will start at [L*cos(theta), L*sin(theta)] with length L2, angle alpha


    
    % BAR 1:
    b1_x0 = 0;
    b1_y0 = 0;
    b1_z0 = 0;

    b1_x1 = L1*cos(theta);
    b1_y1 = L1*sin(theta);
    b1_z1 = b1_z0;

    %BAR 2:
    b2_x0 = b1_x1;
    b2_y0 = b1_y1;
    b2_z0 = b1_z1;


    b2_x1 = b2_x0;
    b2_y1 = b2_y0 + L2*sin(alpha);
    b2_z1 = b2_z0 + L2*cos(alpha);

    P0 = [b1_x0 b1_y0 b1_z0 ; 
          b2_x0 b2_y0 b2_z0];
    P1 = [b1_x1 b1_y1 b1_z1 ; 
          b2_x1 b2_y1 b2_z1];

    X = [P0(:,1) P1(:,1)] ;
    Y = [P0(:,2) P1(:,2)] ;
    Z = [P0(:,3) P1(:,3)] ;
    
    subplot(2,2,plot_number);
    plot3(X', Y', Z','b','LineWidth',2)
    hold on;
    grid on;
    plot3(X',Y',Z','r*')
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    set(gcf,'Position',[100 550 700 700])
    hold off