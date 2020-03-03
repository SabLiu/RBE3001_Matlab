function message = generateMsg(color, size)
if (color == 1)
   color = "blue";
elseif (color == 2)
    color = "green";
else
    color = "yellow";
end
if (size ==0)
   size = "small";
else
    size = "big";
end
str1 = "Holding a ";
str2 = "ball!";
message = str1 + size + " " + color + " "+ str2; 
end