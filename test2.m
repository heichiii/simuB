
R36_sol=[                                0.5, -0.70710678118654752440084436210485,                                0.5;
                               0.5,  0.70710678118654752440084436210485,                                0.5;
-0.70710678118654752440084436210485,                                   0, 0.70710678118654752440084436210485]
 


theta5=atan2(sqrt(R36_sol(3,1)^2+R36_sol(3,2)^2),R36_sol(3,3))
if abs(theta5)>0.00001
    
    theta4=atan2(R36_sol(2,3)/sin(theta5),R36_sol(1,3)/sin(theta5))
    theta6=atan2(R36_sol(3,2)/sin(theta5),-R36_sol(3,1)/sin(theta5))
else
    theta4=0
    theta6=0
end