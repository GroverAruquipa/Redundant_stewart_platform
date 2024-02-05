function ijb=inverse_jacob_hexapod(A1,A2,A3,A4,A5,A6,A7,B1,B2,B3,B4,B5,B6,B7)
    OP=(A1+A2+A3+A4+A5+A6)/6;
    u1=(B1-A1)/(sqrt(sum((B1-A1).^2)));
    u2=(B2-A2)/(sqrt(sum((B2-A2).^2)));
    u3=(B3-A3)/(sqrt(sum((B3-A3).^2)));
    u4=(B4-A4)/(sqrt(sum((B4-A4).^2)));
    u5=(B5-A5)/(sqrt(sum((B5-A5).^2)));
    u6=(B6-A6)/(sqrt(sum((B6-A6).^2)));
    U7=(B7-A7)/(sqrt(sum((B7-A7).^2)));
    r1=A1-OP;
    r2=A2-OP;
    r3=A3-OP;
    r4=A4-OP;
    r5=A5-OP;
    r6=A6-OP;
    r7=A7-OP;
    
% Where u1, u2,u3,u4,u5 are the input vectors calculated with the inverse kinematics
% Where r1,r2,r3,r4 are the vectors of the bar to Op
% 
    Jinv=[u1' (cross(r1,u1))';
    u2' (cross(r2,u2))';
    u3' (cross(r3,u3))';
    u4' (cross(r4,u4))';
    u5' (cross(r5,u5))';
    u6' (cross(r6,u6))';
    U7' (cross(r7,U7))';
 ];

    ijb=Jinv;

end

