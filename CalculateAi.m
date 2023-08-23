function [A_alpha A_a A_theta A_d A_beta]=CalculateAi(iq)
%================================================================
% 功能：  
% 参数：   
%          
% 返回值：  
% 主要思路：
% 备注：    
% 调用方法：
% 日期：    
%================================================================
ialpha=iq(1);
ia=iq(2); 
itheta=iq(3);
id=iq(4);ibeta=iq(5);
c_alpha=cos(ialpha); s_alpha=sin(ialpha);
c_beta=cos(ibeta); s_beta=sin(ibeta);
c_theta=cos(itheta); s_theta=sin(itheta);

A_alpha=[             0       -s_beta*c_theta            -s_theta            -id*s_theta*c_beta;
         s_beta*c_theta                     0     -c_beta*c_theta                   -id*c_theta;
                s_theta        c_beta*c_theta                   0            -id*s_beta*s_theta;
                     0                      0                   0                             0;];
     
 A_a= [0         0         0         c_beta*c_theta;
       0         0         0               -s_theta;
       0         0         0         s_beta*c_theta;
       0         0         0                      0;];
 
 A_theta= [     0           -c_beta         0          0;
              c_beta               0   s_beta          0;
                  0         -s_beta         0          0;
                  0               0         0          0;];
              
     A_d=[     0         0         0          -s_beta;
               0         0         0                0;
               0         0         0           c_beta;
               0         0         0                0;];
           
     A_beta=[  0         0         1                0;
               0         0         0                0;
               -1        0         0                0;
               0         0         0                0;];
           
      
              
             
   