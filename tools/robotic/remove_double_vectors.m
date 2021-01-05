function w = remove_double_vectors(W,tol)
% given a set of unit vectors in W, removes those thate are aligned
% up to a tolerance angle of tol, and gives back them in w
%
% vectors in W are assumed columnwise and get normalized

% default tol
if nargin < 2
     tol = 1e-6;
end
if (tol >= 1) || (tol < 0)
     tol = 1e-6;
end

% normalize W for safety
W = normalize(W,'norm');

% transform tol to cosine
ttol = cos(tol);

% calculate all scalar products betwen columns of W
S = W'*W;

% consider only different vectors, only once
S = tril(S,-1);

% find vector pairs whose dot product is larger than ttol
R = S > ttol;

% rr are the indices of the columns of W to remove
ii = ones(size(R,1),1)*[1:size(R,2)];
rr = unique(reshape(ii(R),1,numel(ii(R))));

% remove columns of W listed in rr
% this part is hig
W(:,rr) = [];

% return w
w = W;

