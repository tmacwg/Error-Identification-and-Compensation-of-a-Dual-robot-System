 

A_a= [0         0         0         c_beta*c_theta;
       0         0         0               -s_theta;
       0         0         0         s_beta*c_theta;
       0         0         0                      0;];
   
   Tibeta=[c_suita*c_beta, -s_suita,c_suita*s_beta,a;
    s_suita*c_alpha*c_beta+s_alpha*s_beta,c_suita*c_alpha,s_suita*c_alpha*s_beta-s_alpha*c_beta, -d*s_alpha;
   s_suita*s_alpha*c_beta-c_alpha*s_beta,c_suita*s_alpha,s_suita*s_alpha*s_beta+c_alpha*c_beta,d*c_alpha;
    0,                              0,                          0,                                     1;];