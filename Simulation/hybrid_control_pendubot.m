function u = hybrid_control_pendubot(x, p, q1_ref_final,q1_ref,dq1_ref,ddq1_ref, kp, kd, K, t,tup_val, SIGMA, LQR)

    q1  = x(1);  q2  = x(2);
    dq1 = x(3);  dq2 = x(4);
    persistent latched
    if isempty(latched) || (nargin >= 20 && resetLatch)
        latched = false;
    end
     q1  = x(1);  q2  = x(2);
    dq1 = x(3);  dq2 = x(4);
    q1_wrapped = mod((q1 )+pi, 2*pi) -pi;
    q2_wrapped = mod((q2 )+pi, 2*pi) -pi;
    x_ref = [q1_ref_final; pi/2 - q1_ref_final; 0 ;0];
    dx1_wrapped = q1_wrapped-x_ref(1);
    dx2_wrapped = q2_wrapped-x_ref(2);
    dx_wrapped = [q1- x_ref(1); dx2_wrapped;dq1;dq2];

    in_LQR_region = (abs(dx1_wrapped )< 0.1) && ...
                    (abs(dx2_wrapped) < 0.5)  && abs(dq2)< 1.0 && abs(dq1) <0.1  && sign(dq2) ~= sign(q2_wrapped); 
    
    

    if ~latched
        if in_LQR_region && LQR
            latched = true;
            % disp(abs(dx1_wrapped ))
            % disp(abs(dx2_wrapped))
            % disp(dx_wrapped)
            % disp(K)
            
        end
    end
   
    if latched
         THETA4 =p.m1*p.lc1 +p.m1*p.l2;


        ueq = THETA4*p.g*cos(q1_ref_final);
        
        u =ueq -K * (dx_wrapped);
        % disp(K)
        % disp(x-x_ref)
        umax =5; 
        u = max( -umax, min(umax, u ));
        if t >10 
            u =0;
        end
        % disp(u)
        return
    end


    m1  = p.m1;  m2  = p.m2;
    lc1 = p.lc1; lc2 = p.lc2;
    l1  = p.l1;
    I1  = p.I1;  I2  = p.I2;
    g   = p.g;

    d11 = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2;
    d12 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
    d21 = d12;
    d22 = m2*lc2^2 + I2;

    h1 = -m2*l1*lc2*sin(q2)*dq2^2 - 2*m2*l1*lc2*sin(q2)*dq1*dq2;
    h2 =  m2*l1*lc2*sin(q2)*dq1^2;

    phi1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    phi2 = m2*lc2*g*cos(q1 + q2);

    D1 =d11 -d12*d21/d22;
    H1 = h1 -d12*h2/d22;
    PHI1 = phi1 -d12*phi2/d22;
    
    if SIGMA ==1
        v1 = kp * (q1_ref_final-q1) - kd * dq1;         % desired ddq1

        
        
        u = D1*v1+PHI1+H1;
        % u = d21 * v1 + d22 * ddq2 + h2 + phi2;

    elseif SIGMA == 2
        if t <tup_val
            v1 = ddq1_ref + kd * (dq1_ref-dq1) +kp * (q1_ref-q1);
        

            
        else
            v1 = kp * (q1_ref_final-q1) - kd * dq1;
        end
        u = D1*v1+PHI1+H1;
    end

end
