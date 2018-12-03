function drawContacts(contacts)
    for i=1:size(contacts, 2)
        plot(contacts(3,i), contacts(4,i), 'ok','LineWidth', 2)
        quiver(contacts(3,i), contacts(4,i), contacts(1,i), contacts(2,i), 'color',[0 0 0],'LineWidth', 2)
        hold on;
    end
end
