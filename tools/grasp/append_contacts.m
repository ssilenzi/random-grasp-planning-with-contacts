function Cpa = append_contacts(Cp1, Cp2)
% APPENDCONTACTS This function appends each contact in Cp2 to Cp1 if they are
% not already in Cp1

Cpa = Cp1;
for i=1:size(Cp2,1)
    if(~isempty(find_row_in_mat(Cpa, Cp2(i,:))))
        continue
    end
    Cpa = [Cpa;Cp2(i,:)];
end
end