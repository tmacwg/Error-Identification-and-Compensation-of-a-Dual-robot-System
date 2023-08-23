function Tk=CalculateT1(qq)
alpha=qq(1);
a=qq(2);
suita=qq(3);
d=qq(4);
beta=qq(5);
c_alpha=cos(alpha);s_alpha=sin(alpha);
c_beta=cos(beta);s_beta=sin(beta);
c_suita=cos(suita);s_suita=sin(suita);
% Tibeta=[c_suita*c_beta, -c_beta*s_suita,s_beta,s_beta*d+a;
%     c_suita*s_alpha*s_beta+c_alpha*s_suita,-s_suita*s_alpha*s_beta+c_alpha*c_suita,-s_alpha*c_beta, -d*s_alpha*c_beta;
%    -c_suita*c_alpha*s_beta+s_alpha*s_suita,s_suita*c_alpha*s_beta+s_alpha*c_suita,c_alpha*c_beta,d*c_alpha*c_beta;
%     0,                              0,                          0,                                     1;];

Tibeta=[                          c_beta*c_suita,                          -c_beta*s_suita,          s_beta,      a + d*s_beta;
c_alpha*s_suita + c_suita*s_alpha*s_beta, c_alpha*c_suita - s_alpha*s_beta*s_suita, -c_beta*s_alpha, -c_beta*d*s_alpha;
s_alpha*s_suita - c_alpha*c_suita*s_beta, c_suita*s_alpha + c_alpha*s_beta*s_suita,  c_alpha*c_beta,  c_alpha*c_beta*d;
                                       0,                                        0,               0,                 1];
Tk=Tibeta;