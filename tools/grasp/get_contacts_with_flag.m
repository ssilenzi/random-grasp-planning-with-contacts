function [success, Cp, Cn] =  get_contacts_with_flag(environment, object, T)
% GETCONTACTS This function collect all contacts and their respective
% normals betwen the object and the environment. It is important to note
% that by default the contacts are reported in the local reference frame
% to the object. If the homogenous transformation T is provided all Contact
% points and normals will be Referenced to T.
% Each contact point is reported in the columns od Cn while the normals in
% the rows of Cn
success = false;
Cp = []; % Initialize variable. A maximum number of contacts is supposed 
         % to be 100
Cn = [];
for i=1:length(environment)
    % Try catch because get_contacts_and_normals_box_box has errors when
    % edge-edge contact (TODO: Solve it)
    try
        [Cpi, Cni] = get_contacts_and_normals_box_box(environment{i}, object);
    catch
        warning('Problem using get_contacts_and_normals_box_box. Exiting.');
        return;
    end
    [Cp, Cn] = append_contacts_and_normals(Cp, Cn, Cpi, Cni);
end

% At this point everyting is ok
success = true;

if exist('T', 'var')
    Cp = transform_points(Cp, T);
    Cn = transform_vectors(Cn,T);
else
    return;
end
end

function [Cp, Cn] = append_contacts_and_normals(Cp1, Cn1, Cp2, Cn2)
Cp = Cp1;
Cn = Cn1;
for i=1:size(Cp2,1)
    p =find_row_in_mat(Cp1,Cp2(i,:));
    if (isempty(p))
        Cp1 = [Cp1;Cp2(i,:)];
        Cn1 = [Cn1;Cn2(i,:)];
    else
        n = find_row_in_mat(Cn1(p,:),Cn2(i,:));
        if(isempty(n))
            Cp1 = [Cp1;Cp2(i,:)];
            Cn1 = [Cn1;Cn2(i,:)];
        end
    end
    Cp = Cp1;
    Cn = Cn1; 
end
end