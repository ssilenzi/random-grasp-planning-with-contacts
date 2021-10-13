function ax = frankaShowCollisionTree(rigidBodyTree, rigidBodyCollisionArray, config)
% frankaShowCollisionTree Plot collision objects using pose from 
% rigidbodytree configuration

ax = gca;
rbt = copy(rigidBodyTree);
rbt.DataFormat = 'column';
rbt.show(config);
hold all

bodies = [{rbt.Base} rbt.Bodies];
for i = 1:numel(bodies)
    % Get tree pose
    TForm = getTransform(rbt, config, bodies{i}.Name, rbt.Base.Name);
    
    % Get collision object information
    collisionObject = rigidBodyCollisionArray{i,1};
    collisionObjectPosition = rigidBodyCollisionArray{i,2};
    
    % Collision object position is a combination of the joint
    % position and the relative pose of the object to the
    % joint.
    if ~isempty(collisionObject)
        collisionObject.Pose = TForm*collisionObjectPosition;
        collisionObject.show('Parent',gca);
    end
end

hold off
end

