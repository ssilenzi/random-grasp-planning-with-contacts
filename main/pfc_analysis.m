function [min_W] = pfc_analysis(Cp, Cn, d)
% PFC_ANALYSIS returns the minimal basis that describes all motions of the 
% object that does not violate constraints described by contact points and 
% normals "Cp" and "Cn".
% d is the dimension where rows of Cp and Cn vectors live.

if isempty(Cp)
    min_W = [eye(d), -eye(d)];
    return;
end

if ~exist('d','var')
    d = 6;
end

    G = [];
    N = [];
    G = build_g(Cp);
    N = build_n(Cn);
    NGt = (G * N).';
    [answ,a, ~] = ptest(NGt);
%     if( rank(G * N) == d)
%         h = 0;
%     else
        h = d - rank(G*N);
%     end

    %-%answ == 2       distacco da tutti
    %-%answ == 1       non c'e' blocco totale, ci puo' essere partial
    %-%answ == 0       form closure
    if answ == 0
        disp('FORM CLOSURE GRASP (STRICT)');
    elseif answ > 0
        %-%      I=gram(NGt'); [ri,ci]=size(I); answ=0; Ubar=[];
        %-%      I=II; answ=0; Ubar=[];ci=6;   %%%NON FUNZIONA !!!
        %-%      I=orth(rand(6,6)); answ=0; Ubar=[];ci=6;   %%%NON FUNZIONA !!!
        [rNGt,cNGt] = size(NGt);
        Constr_Null = cell(rNGt,1); %Inizializza cella di matrici di base degli spazi nulli di ciascun vincolo separatamente
        for i = 1:rNGt
            Constr_Null{i} = truncate(null(NGt(i,:),'r')); 
        end
        cell_combs = nchoosek(1:rNGt, d-h-1);
        % C = nchoosek(v,k) returns a matrix containing all possible combinations of
        % the elements of vector v taken k at a time. Matrix C has k
        % columns and n!/((n�k)! k!) rows, where n is length(v).
        %% Calcola le intersezioni delle combinazioni di spazi nulli d-1 alla volta

        W = [];
        if(~isempty(cell_combs))
            for i = 1:size(cell_combs,1),
                Wi = Constr_Null{cell_combs(i,1)};
                for k = 2:d-h-1
                    Wi = ss_intersect(Wi,Constr_Null{cell_combs(i,k)});
                    %         Wi = SS_intersect(Wi,Constr_Null{cell_combs(i,1+k)});
                end
                W = [W Wi];
            end
        else
            for i = 1:size(Constr_Null,2)
                Wi = Constr_Null{i};
                W = [W Wi];
            end
            W = [W -W]; %% ???
            for i = 1:size(Cn,1)
                W = [W build_g2(Cp(i,:))*Cn(i,:).'];
            end
        end

        %% Prende base positiva del cono libero
        W = truncate(W);
        [rW, cW] = size(W);
        i = 1;
        while (i <= cW)
            %for i=1:cW
            NGtW = truncate(NGt*W(:,i));
            if min(NGtW) >= 0
                i = i+1;
                continue;
            end
            if min(-NGtW) >= 0
                W(:,i) = -W(:,i);
                i = i+1;
                continue;
            end
            W(:,i) = [];
            cW = cW -1;
        end

        %% Toglie colonne che sono comb. lineari positive di altre
        min_W = find_min_positive_base(W);
        % figure
        % plot3(W(1,:), W(2,:), W(3,:), '*')
        if (rank(min_W) == d)
            disp('Constraints are Strongly Accessible and Detachable')
        else
            disp('Constraints are Accessible and Detachable')
        end
    end
end