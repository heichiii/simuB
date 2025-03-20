clear
alpha0=0;
a0=0;
alpha=[ pi/2 0 -pi/2 pi/2 -pi/2];
a=[  0 0.345 0 0 0];
d=[0 0 -0.09 0.295 0 0];
test=[0 pi/8 -pi pi/3 pi/6 pi/4];

syms s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 s6 c6

ca0=1;
sa0=0;
ca1=0;
sa1=1;
ca2=1;
sa2=0;
ca3=0;
sa3=-1;
ca4=0;
sa4=1;
ca5=0;
sa5=-1;


T01=[c1 -s1 0 a0;
    s1*ca0 c1*ca0 -sa0 -sa0*d(1);
    s1*sa0 c1*sa0 ca0 ca0*d(1);
    0 0 0 1];
T12=[c2 -s2 0 a(1);
    s2*ca1 c2*ca1 -sa1 -sa1*d(2);
    s2*sa1 c2*sa1 ca1 ca1*d(2);
    0 0 0 1];
T23=[c3 -s3 0 a(2);
    s3*ca2 c3*ca2 -sa2 -sa2*d(3);
    s3*sa2 c3*sa2 ca2 ca2*d(3);
    0 0 0 1];
T34=[c4 -s4 0 a(3);
    s4*ca3 c4*ca3 -sa3 -sa3*d(4);
    s4*sa3 c4*sa3 ca3 ca3*d(4);
    0 0 0 1];
T45=[c5 -s5 0 a(4);
    s5*ca4 c5*ca4 -sa4 -sa4*d(5);
    s5*sa4 c5*sa4 ca4 ca4*d(5);
    0 0 0 1];
T56=[c6 -s6 0 a(5);
    s6*ca5 c6*ca5 -sa5 -sa5*d(6);
    s6*sa5 c6*sa5 ca5 ca5*d(6);
    0 0 0 1];

T04=T01*T12*T23*T34;
T04=simplify(T04);
R04=T04(1:3,1:3);
R04=simplify(R04)