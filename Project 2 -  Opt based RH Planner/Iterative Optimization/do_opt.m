function x = do_opt(k,ITER,CONSTANTS,T_nom)

    N = CONSTANTS.N;
    n = CONSTANTS.n;
    h_size = CONSTANTS.h_size;

    x0 = [linspace(ITER.pinit(1),ITER.pfin(1),min(h_size, N/n-k+1)*n+1)';
        linspace(ITER.pinit(2),ITER.pfin(2),min(h_size, N/n-k+1)*n+1)';
        min(h_size, N/n-k+1)*(T_nom/(N/n))];
%     x0 = [linspace(ITER.pinit(1),ITER.pfin(1),min(h_size, N/n-k+1)*n+1)';
%         linspace(ITER.pinit(2),ITER.pfin(2),min(h_size, N/n-k+1)*n+1)';
%         10];

    A = []; b = []; Aeq = []; beq = []; lb = []; ub = [];

    options = optimoptions(@fmincon,'Algorithm','sqp',...
        'MaxFunctionEvaluations',300000);
    [x,f] = fmincon(@(x)costfun(x, CONSTANTS)...
        ,x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,ITER,CONSTANTS),options);

end