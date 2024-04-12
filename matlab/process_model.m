function x_k1 = process_model(in1,in2,dt)
%PROCESS_MODEL
%    X_K1 = PROCESS_MODEL(IN1,IN2,DT)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    12-Apr-2024 16:29:07

a1 = in1(1,:);
a2 = in1(2,:);
a3 = in1(3,:);
q1 = in1(4,:);
q2 = in1(5,:);
q3 = in1(6,:);
q4 = in1(7,:);
theta1 = in1(11,:);
theta2 = in1(12,:);
theta3 = in1(13,:);
theta4 = in1(14,:);
theta_dot1 = in1(15,:);
theta_dot2 = in1(16,:);
theta_dot3 = in1(17,:);
theta_dot4 = in1(18,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
w1 = in1(8,:);
w2 = in1(9,:);
w3 = in1(10,:);
t2 = w1.^2;
t3 = w2.^2;
t4 = w3.^2;
t5 = dt.*2.5e+1;
t6 = -t5;
t8 = t2+t3+t4;
t7 = exp(t6);
t9 = sqrt(t8);
t10 = 1.0./t9;
t11 = (dt.*t9)./2.0;
t12 = cos(t11);
t13 = sin(t11);
x_k1 = [a1.*t7;a2.*t7;a3.*t7;q1.*t12-q2.*t10.*t13.*w1.*2.0-q3.*t10.*t13.*w2.*2.0-q4.*t10.*t13.*w3.*2.0;q2.*t12+q1.*t10.*t13.*w1.*2.0+q3.*t10.*t13.*w3.*2.0-q4.*t10.*t13.*w2.*2.0;q3.*t12+q1.*t10.*t13.*w2.*2.0-q2.*t10.*t13.*w3.*2.0+q4.*t10.*t13.*w1.*2.0;q4.*t12+q1.*t10.*t13.*w3.*2.0+q2.*t10.*t13.*w2.*2.0-q3.*t10.*t13.*w1.*2.0;w1;w2;w3;theta1+dt.*theta_dot1;theta2+dt.*theta_dot2;theta3+dt.*theta_dot3;theta4+dt.*theta_dot4;theta_dot1.*(3.0./4.0)+u1./4.0;theta_dot2.*(3.0./4.0)+u2./4.0;theta_dot3.*(3.0./4.0)+u3./4.0;theta_dot4.*(3.0./4.0)+u4./4.0];
