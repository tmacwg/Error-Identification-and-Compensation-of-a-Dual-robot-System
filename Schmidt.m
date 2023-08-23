function B = Schmidt(A)

% input:A=[a1,a2,...ap]

% output:Schmitt-orthogonalized matrix

[n,p]=size(A);

B=zeros(n,p);

% 根据正交化的计算公式

for i=1:n

    B(:,i)=A(:,i);  % bi的初始值

    for j=1:i-1

        B(:,i)=B(:,i)-A(:,i)'*B(:,j)/norm(B(:,j))^2*B(:,j);  % bi

    end

    % 单位化

    B(:,i)=B(:,i)/norm(B(:,i));

end