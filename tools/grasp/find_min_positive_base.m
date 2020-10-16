function W = find_min_positive_base(W0)
% This function find a basis for the positive linear combination of the
% vectors of W0

W=W0;
options = optimoptions('linprog', 'Algorithm', 'interior-point-legacy', 'Preprocess', 'none');
options.Display = 'off';
a = 0;
while(a == 0)
    [~,cW]=size(W);
    for i=1:cW,
        b=W(:,i); A=W; A(:,i)=[]; [~,ca]=size(A);
        if isempty(A)
            return;
        end
        [~,~,flag]=linprog(zeros(ca,1),[],[],A,b,zeros(ca,1),[],[],options);
        %try with orth
        if flag >0
            W=A;
            break; 
        end
    end
    if(i == cW)
        a = 1; 
    end
end