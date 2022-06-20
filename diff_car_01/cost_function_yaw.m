function J = cost_function_yaw(X,U,Xref,ts,Q,R)
    J = 0;
    for i = 1 : length(U)
        u = U(:,i);
%         J = J + transpose(Xref - X)*Q*(Xref - X) + u*R*u; % since in this
%         case we don't need to optimize inputs/controls so we don't have
%         "u*R*u" this term
        J = J + transpose(Xref - X)*Q*(Xref - X);
        X = X + ts*[u(1); u(2); u(3)];
    end
end