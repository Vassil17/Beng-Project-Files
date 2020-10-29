classdef Node
   properties
       position
       parent
       f
       g
       h
   end
    methods
       function obj = Node(Pos,Parent)
           obj.position = Pos;
           obj.parent = Parent;
           obj.f = 0;
           obj.g = 0;
           obj.h = 0;
       end
    end
end