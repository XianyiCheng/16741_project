function [CP, CN] = randomSampleContact(object, env)
% randomly sample a contact point on the object
% CP: contact point, 2 by 1 vector
% CN: contact normal, pointing invard to the object convex hull
% CP and CN are in object frame
% the point shouldn't be sampled on the line segment of the object-environment contacts