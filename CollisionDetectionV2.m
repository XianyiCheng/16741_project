function [bCD,contacts] = CollisionDetectionV2(env, object, config)
[bCD, contacts] = CollisionDetection(env, object, config);
[b, c] = CollisionDetection(fliplr(env), fliplr(object), config);
bCD = bCD || b;
contacts = unique([contacts, c]', 'rows', 'stable')';
if ~isempty(contacts)
    bCD = 0;
end
end