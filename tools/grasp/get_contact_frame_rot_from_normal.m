function [Rot,xc,yc] = get_contact_frame_rot_from_normal(nc)
% GET CONTACT FRAME ROT FROM NORMAL
%   Given a contact normal nc (in a frame, usually global), two orthogonal
%   vectors xc and yc are build an form a contact frame. The rotation
%   matrix is build and returned

%   Attention! nc should be given as row and xc and yc are returned as rows

% getting the orthonormal vectors of nc using null
XY = null(nc);
xc = XY(:,1).';
yc = XY(:,2).';

% Getting the rotation matrix
Rot = [xc.', yc.', nc.'];

end

