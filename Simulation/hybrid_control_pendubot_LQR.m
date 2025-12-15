function u = hybrid_control_pendubot_LQR(x, p, q1_ref_final,K)

    q1  = x(1);  q2  = x(2);
    dq1 = x(3);  dq2 = x(4);
    q1_wrapped = mod((q1 )+pi, 2*pi) -pi;
    q2_wrapped = mod((q2 )+pi, 2*pi) -pi;
    x_ref = [q1_ref_final; pi/2 - q1_ref_final; 0 ;0];
    dx1_wrapped = q1_wrapped-x_ref(1);
    dx2_wrapped = q2_wrapped-x_ref(2);
    dx_wrapped = [q1- x_ref(1); dx2_wrapped;dq1;dq2];
    THETA4 =p.m1*p.lc1 +p.m1*p.l2;


        u_eq = THETA4*p.g*cos(q1_ref_final);
        u =0 -K * (dx_wrapped);
        % disp(K)
        % disp(dx_wrapped)
        umax =5;
        u = max( -umax, min(umax, u ));
        % disp(u)
        % 

  
end
