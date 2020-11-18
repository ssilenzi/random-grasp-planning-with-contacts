function box_out = augment_face(box, i_face, width_augment)
%AUGMMENTFACE This function creates a box from a plane defined by the
% i-th face of the box. width_argument defines the size of the box along
% the width
if ~exist('width_augment','var')
    width_augment = 0.1;
end

switch i_face
    case 1
        T = box.T;
        aug = [box.l/2 - width_augment/2, 0, 0].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(width_augment, box.w, box.h, T);
    case 2
        T = box.T;
        aug = [-box.l/2 + width_augment/2, 0, 0].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(width_augment, box.w, box.h, T);
    case 3
        T = box.T;
        aug = [0, box.w/2 - width_augment/2, 0].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(box.l, width_augment, box.h, T);
    case 4
        T = box.T;
        aug = [0, -box.w/2 + width_augment/2, 0].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(box.l, width_augment, box.h, T);
    case 5
        T = box.T;
        aug = [0, 0, box.h/2 - width_augment/2].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(box.l, box.w, width_augment, T);
    case 6
        T = box.T;
        aug = [0, 0, -box.h/2 + width_augment/2].';
        T(1:3,4) = T(1:3,4) + (T(1:3,1:3))*aug;
        box_out = build_box(box.l, box.w, width_augment, T);
    otherwise
        box_out = [];
        disp('Faces go from 1 to 6. Returning null.');
end
end