function [] = display_x_dot(x_dot)
    disp("omega dot");
    disp(x_dot(1:3, :));

    disp("phi dot");
    disp(x_dot(4:6, :));

    disp("v dot");
    disp(x_dot(7:9, :));

    disp("p dot");
    disp(x_dot(10:12, :));
end