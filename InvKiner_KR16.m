function oAngx=InvKiner_KR16(Q,iT)
%初始化
%得到的角度没有用到基准角度，没有用到
global ERRAng;
ERRAng=981.5555;
%        alpha         a             theta         d            beta   
%    Q0 =  [0         0         0      780/1000      0
%         -pi/2    320/1000   -pi/2       0           0
%          0      1125/1000      0          0            0
%        -pi/2      200/1000     -pi       1142.5/1000      0
%        -pi/2         0         0         0            0
%        -pi/2        0        pi      200/1000        0];% 6700-200/2.60关节参数
   
% Q0 =  [pi         0         0    -520/1000      0
%     pi/2     160/1000      0         0         0
%     0        780/1000    -pi/2       0         0
%     pi/2     150/1000      0    -655/1000      0
%     -pi/2        0         0         0         0
%     pi/2         0         0     -153/1000     0]; % KUKA KR16 R1610 的DH模型  

Q0 =  [0         0         0       520/1000      0
    -pi/2     160/1000     -pi/2     0           0
    0        1080/1000      0         0           0
    -pi/2     150/1000      -pi    655/1000      0
    -pi/2        0           0       0           0
    -pi/2         0         pi     153/1000     0]; % KUKA KR16 R1610 的DH模型  



%   Q0 =  [0         0         0    814.5/1000         0
%         -pi/2    300/1000   -pi/2         0         0
%          0    700/1000         0         0         0
%        -pi/2    280/1000    pi    893/1000         0
%        pi/2         0         0         0         0
%        -pi/2         0         pi    200/1000      0];


%      Q0 =  [0         0         0      486.5/1000      0
%         -pi/2    150/1000   -pi/2       0           0
%          0       475/1000      0          0            0
%        -pi/2        0        -pi       600/1000      0
%        -pi/2         0         0         0            0
%        -pi/2        0        pi      65/1000        0];% 1600-10/1.2关节参数
d1=Q0(1,4); d4=Q0(4,4); d6=Q0(6,4);
a1=Q0(2,2);a2=Q0(3,2);a3=Q0(4,2);
nx=iT(1,1); ox=iT(1,2);ax=iT(1,3); px=iT(1,4);
ny=iT(2,1); oy=iT(2,2);ay=iT(2,3);py=iT(2,4);
nz=iT(3,1); oz=iT(3,2);az=iT(3,3);pz=iT(3,4); 
l1=ay*d6-py;l2=ax*d6-px;
sta1_1=atan2(ay*d6-py,ax*d6-px);%另外一个解集合是
sta1_2=atan2(ay*d6-py,ax*d6-px)-pi;
Theta1=[sta1_1 sta1_2];
Theta3=[];
for i=1:2
    sta1=Theta1(1,i);
c1=cos(sta1);s1=sin(sta1);
B=px*c1+py*s1-(ax*c1+ay*s1)*d6-a1;C=pz-d1-az*d6;
K=(B*B+C*C-a2*a2-a3*a3-d4*d4)/(2*a2);
lv=(a3*a3+d4*d4)^0.5;
%t1=(1-K^2/lv^2)^0.5;t2=K/lv;a2x=atan2((1-K^2/lv^2)^0.5,K/lv);
%sta3_1=atan2((1-K^2/lv^2)^0.5,K/lv)-atan2(d4,a3);
%sta3_2=atan2(-(1-K^2/lv^2)^0.5,K/lv)-atan2(d4,a3);

%去除不可达的无解情况下
testVal=1-K^2/lv^2;
if testVal<0
    sta3_1=ERRAng;
    sta3_2=ERRAng;
else 
    sta3_1=atan2((1-K^2/lv^2)^0.5,K/lv)-atan2(d4,a3);
    sta3_2=atan2(-(1-K^2/lv^2)^0.5,K/lv)-atan2(d4,a3);
end

Theta3=[Theta3 sta3_1 sta3_2];
end
Theta13=[sta1_1 sta1_1 sta1_2 sta1_2;Theta3];


Theta2=[];
for i=1:4
    sta1=Theta13(1,i);
    sta3=Theta13(2,i);
    c1h=cos(-sta1);s1h=sin(-sta1);s3h=sin(sta3+pi/2);c3h=cos(sta3+pi/2);
E=c1h*px-s1h*py-c1h*ax*d6+s1h*ay*d6-a1;
F=pz-d1-d6*az;
G=a3+a2*s3h;
H=d4+c3h*a2;
sta23=atan2(G*E-H*F,G*F+E*H);
sta2=sta23-(sta3+pi/2);
Theta2=[Theta2 sta2];
end
Theta123=[Theta13(1,:);Theta2;Theta13(2,:)];


Theta4=[];
Theta5=[];
Theta6=[];
for i=1:4
    %%求解Theta4
   sta1=Theta123(1,i);
   sta2=Theta123(2,i);
   sta3=Theta123(3,i); 
    c1h=cos(-sta1);s1h=sin(-sta1);s23h=sin(sta2+sta3+pi/2);c23h=cos(sta2+sta3+pi/2);
    tmpx=-s1h*ax-c1h*ay; tmpy=-s23h*c1h*ax+s23h*s1h*ay-c23h*az;
    sta4_1=atan2(-s1h*ax-c1h*ay,-s23h*c1h*ax+s23h*s1h*ay-c23h*az);
    sta4_1=-sta4_1;%与论文中是互为相反数的
    sta4_2=sta4_1+pi;
    Theta4=[Theta4 sta4_1 sta4_2];
    
    
    %求解Theta5
    c4h=cos(-sta4_1);s4h=sin(-sta4_1);%与论文中是互为相反数的
    dx=-(s4h*s1h+c1h*c4h*s23h)*ax-(s4h*c1h-s1h*c4h*s23h)*ay-(c23h*c4h)*az;
    dy=c23h*c1h*ax-c23h*s1h*ay-s23h*az;
    sta5_1=atan2(dx,dy);%与论文中是一样的
    sta5_2=-sta5_1;
    Theta5=[Theta5 sta5_1 sta5_2];
    
    
    %求解Theta6
    c5h=cos(sta5_1);s5h=sin(sta5_1);
    c6=(s4h*c5h*s1h+s5h*c1h*c23h+c5h*c1h*c4h*s23h)*nx+(s4h*c5h*c1h-s5h*s1h*c23h-c5h*s1h*c4h*s23h)*ny+(c5h*c4h*c23h-s5h*s23h)*nz;
    c6=-c6;
    s6=(s1h*c4h-s4h*c1h*s23h)*nx+(c1h*c4h+s4h*s1h*s23h)*ny+(-c23h*s4h)*nz;
    s6=-s6;
    
    sta6_1=atan2(s6,c6);
    sta6_1=-sta6_1-pi;
    sta6_2=sta6_1+pi;
    
    Theta6=[Theta6 sta6_1 sta6_2];
    
end
Theta123456=[Theta123(:,1 ) Theta123(:,1 ) Theta123(:,2 ) Theta123(:,2 ) Theta123(:,3 ) Theta123(:,3 ) Theta123(:,4 ) Theta123(:,4 );Theta4;Theta5;Theta6;];

[Trow Tarray]=size(Theta123456);
for i_Sol=1:Trow
    for j_Sol=1:Tarray
        
        if Theta123456(i_Sol,j_Sol)>pi
            Theta123456(i_Sol,j_Sol)=Theta123456(i_Sol,j_Sol)-2*pi;
        end
          if Theta123456(i_Sol,j_Sol)<-pi
            Theta123456(i_Sol,j_Sol)=Theta123456(i_Sol,j_Sol)+2*pi;
        end
        
    end
end
Theta123456New=[];
for j_Sol=1:Tarray
  if Theta123456(3,j_Sol)<ERRAng+0.00001 && Theta123456(3,j_Sol)>ERRAng-0.00001
    Theta123456(:,j_Sol)=[ERRAng ERRAng ERRAng ERRAng ERRAng ERRAng]'; 
  else
      Theta123456New=[Theta123456New Theta123456(:,j_Sol)];
  end  
end

%%求解theta5
%%%遗留问题1，如果大于正负180度，需要调整到区间内；
%%%如果无解；
%%%如果不满足指定范围；
%%当theta5=0；则theta4是求不出的；2度也可以；
oAngx=Theta123456New;%

sss=1;







oAng=[sta1 sta1_2]';