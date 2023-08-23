a=K1(3*i-2:3*i,:)*Desdq+ MeaTbt1(i*4-3:i*4-1,4); %式3-22第二个式子
b=MeaTbt1(i*4-3:i*4,:)+[-MeaTbt1(i*4-3:i*4-1,1:3)*Antisymmetric(Desbs1(i*4-3:i*4-1,1:3)'*Desdq(4:6)),K1(3*i-2:3*i,:)*Desdq;[0 0 0 0]]; % 式3-21最后一个等号
c=MeaTst1(i*4-3:i*4-1,1:3)*[-eye(3),Antisymmetric(Desbs1(i*4-3:i*4-1,4))]*G1(6*i-5:6*i,:)*Desdq+MeaTbt1(i*4-3:i*4-1,4);% 式3-21第二行的平移部分
d=inv(MeaTts1(i*4-3:i*4,:)+[Antisymmetric(G1(6*i-2:6*i,:)*Desdq),G1(6*i-5:6*i-3,:)*Desdq;[0 0 0 0]]*MeaTts1(i*4-3:i*4,:))*Desbs1(i*4-3:i*4,:); % 式3-19第二行
e=inv(DesTe1s*M1(:,:,i)*DesTo2o1*E1_Des(:,:,i)*DesTte2)*Desbs1(i*4-3:i*4,:); %式3-19第一行右侧
f=inv(MeaTts1(i*4-3:i*4,:)+D2T(G1(6*i-5:6*i,:)*Desdq)*MeaTts1(i*4-3:i*4,:))*Desbs1(i*4-3:i*4,:); % 式3-19第1行

