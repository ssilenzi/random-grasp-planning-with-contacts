function frankaHighlightCollisionsBodies(robot, collisionBodyIdx, ...
    ax, highlightColor)
% This function is for internal use only and may be removed in a future release.

% frankaHighlightCollisionsBodies Check for collisions
%   Highlight the bodies with indices given in the COLLISIONBODYIDX matrix,
%   where the indices start correspond to the following ordering:
%   [robot.Base robot.Bodies]. The visualization occurs in the axes
%   specified by ax, which must already contain a visualization of the
%   associated rigidbodytree, given by ROBOT. The color can be specified.

%   Copyright 2019 The MathWorks, Inc.

if ~exist('highlightColor', 'var')
    highlightColor = [1 0.8 0]; % Yellow
end

% The rigid bodies actually start at the first link, not the
% base, so the indices have to be shifted down be 1
validateattributes(collisionBodyIdx, {'double'}, {}, 'showCollision', 'collisionBodyIdx');
if ~isempty(collisionBodyIdx)
    rigidBodyIdx = collisionBodyIdx-1;
else
    rigidBodyIdx = collisionBodyIdx;
end

for i = 1:numel(rigidBodyIdx)
    if rigidBodyIdx(i) < 0
        % Body is the base
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Base.Name '_mesh']);
    else
        % Any other body
        p = findall(ax, 'type', 'patch', 'displayname', [robot.Bodies{rigidBodyIdx(i)}.Name '_mesh']);
    end
    
    if isempty(p)
        continue
    else
        p(1).FaceColor = highlightColor;
    end
end
end