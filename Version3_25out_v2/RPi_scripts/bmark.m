clear;
close all;
MAC = 'C0:7A:34:31:2D:48';
mypi = raspi('10.42.0.128','pi','raspberry');

dat1 = pt(mypi,MAC);
b=0;
for i=1:100 
    tic;
    dat1.update;
    a=toc;
    b=a+b;
end
display(b/100);

delete(dat1);

dat2 = agm(mypi,MAC);
b=0;
for i=1:100 
    tic;
    dat2.update;
    a=toc;
    b=a+b;
end
display(b/100);
delete(dat2);