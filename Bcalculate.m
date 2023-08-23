function [ B ] = Bcalculate( A )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
[a,b]=size(A);
for i=1:a/3
    p1=A((3*i-2),:);
    p2=A((3*i-1),:);
    p3=A((3*i),:);
    t12=p1-p2;
    t23=p2-p3;
    t31=p3-p1;
    n(i,:)=cross(t12,t23)/norm(cross(t12,t23));
    if n(i,3)>0
        n(i,:)=-n(i,:);
    end
    
    s1=abs(t12*t23'/(norm(t12)*norm(t23')));
    s2=abs(t23*t31'/(norm(t23)*norm(t31')));
    s3=abs(t31*t12'/(norm(t31)*norm(t12')));
    [q,s]=sort([s1,s2,s3]);
    if s(1)==1
        o(i,:)=p2;
        if norm(t23)>norm(t12)
            x(i,:)=[norm(p3-p2),0,0];
            p(i,:)=[(p1-p2)*(p3-p2)'/norm(p3-p2),(p1-p2)*cross(n(i,:),(p3-p2))'/norm(cross(n(i,:),(p3-p2))),(p1-p2)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[p(i,:);o;x(i,:)];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
        %    [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        else
            x(i,:)=[norm(p1-p2),0,0];
            p(i,:)=[(p3-p2)*(p1-p2)'/norm(p1-p2),(p3-p2)*cross(n(i,:),(p1-p2))'/norm(cross(n(i,:),(p1-p2))),(p3-p2)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[x(i,:);o;p(i,:)];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
       %     [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        end
    end
    if s(1)==2
        o(i,:)=p3;
        if norm(t23)>norm(t31)
            x(i,:)=[norm(p2-p3),0,0];
            p(i,:)=[(p1-p3)*(p2-p3)'/norm(p2-p3),(p1-p3)*cross(n(i,:),(p2-p3))'/norm(cross(n(i,:),(p2-p3))),(p1-p3)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[p(i,:);x(i,:);o];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
        %    [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        else
            x(i,:)=[norm(p1-p3),0,0];
            p(i,:)=[(p2-p3)*(p1-p3)'/norm(p1-p3),(p2-p3)*cross(n(i,:),(p1-p3))'/norm(cross(n(i,:),(p1-p3))),(p2-p3)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[x(i,:);p(i,:);o];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
  %          [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        end
    end
    if s(1)==3
        o(i,:)=p1;
        if norm(t31)>norm(t12)
            x(i,:)=[norm(p3-p1),0,0];
            p(i,:)=[(p2-p1)*(p3-p1)'/norm(p3-p1),(p2-p1)*cross(n(i,:),(p3-p1))'/norm(cross(n(i,:),(p3-p1))),(p2-p1)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[o;p(i,:);x(i,:)];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
    %        [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        else
            x(i,:)=[norm(p2-p1),0,0,];
            p(i,:)=[(p3-p1)*(p2-p1)'/norm(p2-p1),(p3-p1)*cross(n(i,:),(p2-p1))'/norm(cross(n(i,:),(p2-p1))),(p3-p1)*n(i,:)'/norm(n(i,:))];
            o=[0,0,0];
            AR((3*i-2):3*i,:)=[o;x(i,:);p(i,:)];
            [R,t] = rigid_transform_3D(A((3*i-2):3*i,:),AR((3*i-2):3*i,:));
   %         [R1,t1]=rigid_transform_3D(AR((3*i-2):3*i,:),A((3*i-2):3*i,:));
            B(4*i-3:4*i,:)=[R,t;[0,0,0,1]];
        end
    end
    B(4*i-3:4*i,:)=inv(B(4*i-3:4*i,:));
    oo=B(4*i-3:4*i,:)*[x(i,:),1]';
    ooo=B(4*i-3:4*i,:)*[p(i,:),1]';
    aaa=inv(B(4*i-3:4*i,:))*[p1,1]';
    bbb=inv(B(4*i-3:4*i,:))*[p2,1]';
    ccc=inv(B(4*i-3:4*i,:))*[p3,1]';
    p33=B(4*i-3:4*i,:)*ccc;
    
    
%     p11=B(4*i-3:4*i,:)*[p1,1]';
%     p22=B(4*i-3:4*i,:)*[p2,1]';
%     p33=B(4*i-3:4*i,:)*[p3,1]';
%     a=norm(t12);
%     b=norm(t23);
%     c=norm(t31);
%     d=norm(p(i,:)-x(i,:));
end




end

