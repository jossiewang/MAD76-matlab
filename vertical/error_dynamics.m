function [x_diff,dist] =error_dynamics(spline,x)
    v_max=0.35;
    s1=x(1);
    s2=x(2);
    vr=x(3);
    psi=x(5);
    s=[s1;s2];

    sc2e_max=0.01;
    psi_e_max=pi;
    
    
    breakslen=spline.breakslen;
    points=spline.points;
    coefs=spline.coefs;
    periodic=spline.periodic;
    [w,~]=mbc_spline_get_reference(s,breakslen,points,coefs,periodic,...
        vr,0);
    psi_star=w(2);
    s_star=[w(3);w(4)];
    sc2e=(s2-s_star(2))*cos(psi_star)-(s1-s_star(1))*sin(psi_star);
    psi_e = atan2(sin(psi - psi_star), cos(psi - psi_star));
    v_diff=(v_max-vr);
    dist=double(w(1));
    x_diff=[sc2e/sc2e_max; psi_e/psi_e_max;v_diff/v_max];

end