function W=Poisson(T,h,lamda)
%%%% 6 Jump
P(1)=0;
for i=1:1:200
    a=-log(1-rand)/lamda;
    P(i+1)=P(i)+a;
end
counter=2;
P=P*(1/h);
P=round(P);
W=0;
for i=1:1:2*T/h
    if counter==201
        W(i)=0;
    end
    if counter<=201
    if i<P(counter)
        W(i)=0;
    end
    if  i==P(counter)
        W(i)=P(counter);
        counter=counter+1;
    end
    end
end
end