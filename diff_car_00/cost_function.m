function J = cost_function(X,U,Xref,theta,ts,Q,R)
    J = 0;
    for i = 1 : length(U)
        u = U(:,i);
%         J = J + transpose(Xref - X)*Q*(Xref - X) + u*R*u; % since in this
%         case we don't need to optimize inputs/controls so we don't have
%         "u*R*u" this term
        J = J + transpose(Xref - X)*Q*(Xref - X);
        theta = theta + ts*u(2);
        X = X + ts*u(1)*[cos(theta); sin(theta)];
    end
end