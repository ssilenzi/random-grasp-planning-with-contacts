function b = copy_object(a)

% This function is for copying a class. As a class is a handle, assignment
% won't work...
% https://www.mathworks.com/matlabcentral/answers/ ...
%   312653-matlab-object-assignment-copy-an-object-instead-of-creating-a-pointer
% TODO: Implement the following for copying
% https://www.mathworks.com/help/matlab/matlab_oop/custom-copy-behavior.html

   b = eval(class(a));  %create default object of the same class as a. one valid use of eval
   for p =  properties(a).'  %copy all public properties
      try   %may fail if property is read-only
         b.(p) = a.(p);
      catch
         warning('failed to copy property: %s', p);
      end
   end
end

