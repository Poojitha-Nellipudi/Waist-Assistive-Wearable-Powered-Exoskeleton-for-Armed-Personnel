function [q, prof_bp] = inverse_kinematic(T06,J06,mu,q)
%Function takes required position mu and initial assumed angles
%Returns the correct angles using Newton-Raphson method

syms a0 a1 a2 d0 q1 q2 q3 q4 q5 qg real ;
    

for i = 1:100
   % q1 = q(1,i)
   % q2 = q(2,i); 
   % q3 = q(3,i);  
   
    % Forward kinematic model
    mu_e = [simplify(T06(1,4)) ; simplify(T06(2,4)) ; simplify(T06(3,4))]; %planar
    vars = [a0, a1, a2, d0, q1, q2, q3, q4, q5];
    newVars = [0.1, 0.3, 0.3, 0.05, q(1,i), q(2,i), q(3,i), q(4,i), q(5,i)];
    mu_e = subs(mu_e,vars, newVars);
    
    % Error estimation
    del_mu = vpa(mu - mu_e);
    
    % Jacobain matrix
    J06_1= subs(J06,vars, newVars);
   
    % Newton method
    q(:,i+1) = q(:,i) + vpa(simplify(pinv(J06_1)))* del_mu;
    
    % Termination condition
    if abs(del_mu(1))<=1e-5 && abs(del_mu(2))<=1e-5 && abs(del_mu(3))<=1e-5
        qf = q(:,i+1);
        break
    end
end

    
    q = vpa(qf);
    
    prof_bp = vpa(mu_e);
    
 
    prof_bp = [prof_bp(1);prof_bp(3)];

end

