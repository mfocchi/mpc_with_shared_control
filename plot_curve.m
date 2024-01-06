function  plot_curve(log_state, ref_state, p0, params, DEBUG)

%initial REF
plot(p0(1), p0(2) , 'Marker', '.', 'Color','g', 'MarkerSize',60) ; hold on;

% plot ref trajectory
plot(ref_state(1,:), ref_state(2,:), 'ro-',...
    'LineWidth', 1.0,'MarkerSize',4);

%initial ACT
plot(log_state(1,1), log_state(2,1) , 'Marker', '.', 'Color','g', 'MarkerSize',60) ; hold on;


% plot actual trajectory
plot(log_state(1,:), log_state(2,:), 'ko-',...
    'LineWidth', 1.0,'MarkerSize',4);

% plot obstacle
pos = params.obstacle_pos;
r = params.obstacle_radius;
th = linspace(0,2*pi*100);
x = cos(th) ; y = sin(th) ;
plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);hold on;
plot(pos(1), pos(2), '.', 'MarkerSize', 50, 'LineWidth', 20);

%limits
% min_x = min(solution.mpc_states(1,:)) ;
% max_x = max(max(solution.p(1,:)), pf(1))+3 ;
% min_y = min(min(solution.p(2,:)), pf(2))-3 ;
% max_y = max(max(solution.p(2,:)), pf(2)) +3 ;
%set(gca,'XLim',[min_x max_x])
%set(gca,'YLim',[min_y max_y])


color_input = 'r'; 

scaling = 0.05;
%initial orient
plotOrientation([p0(1); p0(2)], p0(3), scaling);

if DEBUG
    for i =1:length(log_state)
        plotOrientation([log_state(1,i); log_state(2,i)], log_state(3,i), scaling);
    end
end


grid on;
axis equal


xlabel('X');
ylabel('Y');



end