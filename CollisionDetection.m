function [bCD,contacts] = CollisionDetection(env, object, X)
% X: object configuration
% return:
% bCD: if collide, boolean. If object is in contact with env, bCD = 0
% contacts: 4 x ? matrix. contact = [contact normal, contact location]' ,
% if there is no contact, contacts = [];

contacts = [];