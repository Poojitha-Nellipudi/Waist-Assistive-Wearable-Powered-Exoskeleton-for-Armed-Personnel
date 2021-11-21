function biped_c = animation_plot(q,q_new,T01,T02,T03,T04,T05,T06)
    

    x1 = T01(1,4);
    y1 = T01(3,4);
    x2 = T02(1,4);
    y2 = T02(3,4);
    x3 = T03(1,4);
    y3 = T03(3,4);
    x4 = T04(1,4);
    y4 = T04(3,4);
    x5 = T05(1,4);
    y5 = T05(3,4);
    x6 = T06(1,4);
    y6 = T06(3,4);


    x2 = subs(x2,q,q_new);
    y2 = subs(y2,q,q_new);
    x3 = subs(x3,q,q_new);
    y3 = subs(y3,q,q_new);
    x4 = subs(x4,q,q_new);
    y4 = subs(y4,q,q_new);
    x5 = subs(x5,q,q_new);
    y5 = subs(y5,q,q_new);
    x6 = subs(x6,q,q_new);
    y6 = subs(y6,q,q_new);
    
    biped_c =  [0,x1,x2,x3,x4,x5,x6;
                0,y1,y2,y3,y4,y5,y6];

end

