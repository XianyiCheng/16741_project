function drawContacts(contacts)
    for i=1:size(contacts, 2)
        plot(contacts(3,i), contacts(4,i), 'ok')
        quiver(contacts(3,i), contacts(4,i), contacts(1,i), contacts(2,i), 'color',[0 0 0])
    end
end