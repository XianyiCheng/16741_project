function [bCD,contacts] = CollisionDetectionV2(env, object, config)
[bCD, contacts] = CollisionDetection(env, object, config);
[b, c] = CollisionDetection(fliplr(env), fliplr(object), config);
bCD = bCD || b;
contacts = unique([contacts, c]', 'rows', 'stable')';
if ~isempty(contacts)
    bCD = 0;
    ret = contacts(:,1);
    for i=2:size(contacts,2)
        flag = false;
        for j=1:size(ret, 2)
            if norm(contacts(:,i)-ret(:,j)) <= 1e-3
                flag = true;
            end
        end
        if flag
            ret = [ret, contacts(:,i)];
        end
    end
end
end