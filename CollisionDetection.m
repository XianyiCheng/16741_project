function [bCD,contacts] = CollisionDetection(env, object, config)
% X: object configuration (Assume object X is a N-vertice polygon (2xN), environment env is a M-vertice polygon (2xM))
% return:
% bCD: if collide, boolean. If object is in contact with env, bCD = 0
% contacts: 4 x ? matrix. contact = [contact normal, contact location]' ,
% if there is no contact, contacts = [];
    X = objFrame2worldFrame(object, config);
    contacts = [];
    bCD = 0;
    % wrap around
    lines_X = [X', [X(:,2:end)'; X(:,1)']];
    lines_env = [env', [env(:,2:end)'; env(:,1)']];
    
    out = lineSegmentIntersect(lines_X, lines_env);
    
    for N1=1:size(lines_X,1)
        for N2=1:size(lines_env,1)
            if out.intAdjacencyMatrix(N1, N2)
                % let contact normal be (0,0) if two line segments are
                % "crossing" each other
                contact_point = [out.intMatrixX(N1,N2); out.intMatrixY(N1,N2)];
                if out.intNormalizedDistance1To2(N1,N2) == 0
                    line = lines_env(N2,:);
                    if line(3) == line(1)
                        normal = [1; 0];
                    else
                        slope = (line(4)-line(2))/(line(3)-line(1));
                        if slope == 0
                            normal = [0; 1];
                        else
                            normal = [1; -1/slope];
                            normal = normal/norm(normal);
                        end
                    end
                    test_point = contact_point+normal/1000;
                    if ~inpolygon(test_point(1), test_point(2), env(1,:), env(2,:))
                        normal = -normal;
                    end
                    contacts = [contacts, [normal;contact_point]];
                end
                if out.intNormalizedDistance2To1(N1,N2) == 0
                    line = lines_X(N1,:);
                    if line(3) == line(1)
                        normal = [1; 0];
                    else
                        slope = (line(4)-line(2))/(line(3)-line(1));
                        if slope == 0
                            normal = [0; 1];
                        else
                            normal = [1; -1/slope];
                            normal = normal/norm(normal);
                        end
                    end
                    test_point = contact_point+normal/1000;
                    if ~inpolygon(test_point(1), test_point(2), env(1,:), env(2,:))
                        normal = -normal;
                    end
                    contacts = [contacts, [normal;contact_point]];
                end
                if out.intNormalizedDistance1To2(N1,N2) ~= 0 && out.intNormalizedDistance2To1(N1,N2) ~= 0
                    bCD = 1;
                end
            end
        end
    end

if ~isempty(contacts)
    bCD=0;
end
    
    
    
