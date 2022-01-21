clc;clear;
syms f1 f2 f3 f4 f5 f6 f7 f8 
syms p1 p2 p3 p4 p5 p6 p7 p8
syms s c 
A = [...
    f1 p1 zeros(1,4);
    f2 p2 zeros(1,4);
    f3 p3 zeros(1,4);
    f4 p4 zeros(1,4);
    f5 p5 zeros(1,4);
    f6 p6 zeros(1,4);
    f7 p7 zeros(1,4);
    f8 p8 zeros(1,4);
    zeros(4,2),eye(4)
    ];

S3 = [...
    zeros(4,2),eye(4),[zeros(2);eye(2)],eye(4) 
];


S3A = S3*A;


G = [...
    c,0;
    s,0;
    0,1
    ];

S4 = [...
    eye(2),zeros(2,4)
    ];


GS = G*S4;