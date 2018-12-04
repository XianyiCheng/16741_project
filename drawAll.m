function drawAll(env,object, config, contacts)
drawEnv(env);
hold on;
drawObject(objFrame2worldFrame(object, config));
hold on;
drawContacts(contacts);