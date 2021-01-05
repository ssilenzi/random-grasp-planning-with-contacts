function [Rot,xc,yc,zc] = get_contact_frame_rot_from_normal(nc)
% GET CONTACT FRAME ROT FROM NORMAL
%   Given a contact normal nc (in a frame, usually global), two orthogonal
%   vectors xc and zc are build and form the contact frame. The rotation
%   matrix from global to finger frame is build and returned

% Here, we suppos that the extension of the finger is given by the y axis
% of the finger frame. This should be taken into account while designing
% the robot class

%   Attention! nc should be given as row and xc and zc are returned as rows

yc = nc/norm(nc); % normalizing

% getting the orthonormal vectors of nc using null
XY = null(yc);
xc = XY(:,1).';
zc = XY(:,2).';

% To get a right hand frame
if norm(xc - cross(yc,zc)) > 0.1 
    xc = -xc;
end

% Getting the rotation matrix
Rot = [xc.', yc.', zc.'].';

end

