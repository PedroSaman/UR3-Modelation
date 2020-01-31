for i=1:length(q1)
    tb1(i) = cos(q2(i) + q1(i))*g*lc2*m2 + cos(q2(i))*l1*lc2*m2*ddq1(i) + cos(q2(i))*l1*lc2*m2*ddq2(i) + sin(q2(i))*l1*lc2*m2*dq1(i)^2 - sin(q2(i))*l1*lc2*m2*dq2(i)^2 + (g*l1*m2 + g*lc1*m1)*cos(q1(i)) + (l1^2*m2 + lc1^2*m1 + I1)*ddq1(i) + (lc2^2*m2 + I2)*ddq2(i) + b1*dq1(i) + b2*dq2(i);
    tb2(i) = cos(q2(i))*l1*lc2*m2*ddq1(i) + (lc2^2*m2 + I2)*ddq2(i) + sin(q2(i))*l1*lc2*m2*dq1(i)^2 + cos(q2(i) + q1(i))*g*lc2*m2 + b2*dq2(i);
end