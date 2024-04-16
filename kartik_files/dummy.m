function y = dummy()

    persistent yyy yyyy
    for i = 1:1
    if isempty(yyy)
        yyy = 100;
    else
        yyy = 1000 + yyyy;
    end
    yyyy = 1;
    end
    y = yyy
end