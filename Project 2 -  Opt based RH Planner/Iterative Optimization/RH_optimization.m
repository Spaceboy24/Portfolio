function [x_opt, y_opt, t_opt] = RH_optimization(x_nom, y_nom, t_nom)

load_parameters;

N = CONSTANTS.N;
n = CONSTANTS.n;
h_size = CONSTANTS.h_size;

% [x_nom, y_nom, t_nom] = get_init();

T_nom = t_nom;

for k = 1:N/n

    ITER.pinit = [x_nom(k,1), y_nom(k,1)];
    ITER.pfin = [x_nom(min(k+h_size-1,N/n), end), y_nom(min(k+h_size-1,N/n), end)];
    ITER.h_size = min(h_size, N/n-k+1);
    
    x = do_opt(k,ITER,CONSTANTS,T_nom);
    
    x_h = [];
    y_h = [];    
    for j = 0:ITER.h_size-1

        x_h(j+1,:) = x(n*j+1:n*(j+1)+1);
        y_h(j+1,:) = x(ITER.h_size*n+n*j+2:ITER.h_size*n+n*(j+1)+2);

    end
    T = x(end);
    dt = T_nom/(N/n) - T;
    
    x_nom(k:min(k+h_size-1,N/n),:) = x_h(:,:);
    y_nom(k:min(k+h_size-1,N/n),:) = y_h(:,:);
    T_nom = T_nom - dt;
    
    plot(ITER.pinit(1), ITER.pinit(2),'o'), hold on
    plot(ITER.pfin(1), ITER.pfin(2),'o'), hold on
    
    
end

for k = 1:size(x_nom)
   
    x_p = x_nom(k,:);
    y_p = y_nom(k,:);
    tnodes = (k-1)*(T/n):0.01:k*(T/n);
    c = [BernsteinPoly(x_p, tnodes); BernsteinPoly(y_p, tnodes)];
    plot(c(1, :), c(2, :)), hold on
    plot(x_p,y_p,'*'), hold on
    xlabel('x');
    ylabel('y');

end

x_opt = x_nom;
y_opt = y_nom;
t_opt = T_nom;
end

