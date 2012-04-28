clc;
k=1;
obj = videoinput('winvideo', 2);
set(obj,'ReturnedColorSpace', 'RGB');
preview(obj);
for m=1:k;
I= getsnapshot(obj);    %capturing an image of the arena from the overhead camera

s = serial('COM16');   % DEFINE NAME TO SERIAL(for red robot)
set (s, 'BAUDRATE',115200,'databits',8,'parity','none',...
    'stopbits',1,'FlowControl','none','InputBufferSize', 1);
set(s, 'Timeout', 20);
fopen(s);

z = serial('COM1');   % DEFINE NAME TO SERIAL(for blue robot)
set (z, 'BAUDRATE',9600,'databits',8,'parity','none',...
    'stopbits',1,'FlowControl','none','InputBufferSize', 1);
set(z, 'Timeout', 20);
fopen(z);

cnt1=0;
cnt2=0;

I1=I(:,:,1)
I2=I(:,:,2)
I3=I(:,:,3)
[m n]=size(I3);
imtool(I);

    %identifying the red pixels and making them 1 and the rest of the pixels 0
for i=1:1:m
    for j=1:1:n
        if(I1(i,j)>220)
           I1(i,j)=1;
        else
            I1(i,j)=0;
        end
        if(I1(i,j)>220&&I2(i,j)>220&&I3(i,j)>220)
            I1(i,j)=0;
        end
    end
end

    %identifying the blue pixels and making them 1 and the rest of the pixels 0
for i=1:1:m
    for j=1:1:n
        if(I3(i,j)>220)
           I3(i,j)=1;
        else
            I3(i,j)=0;
        end
        if(I1(i,j)>220&&I2(i,j)>220&&I3(i,j)>220)
            I3(i,j)=0;
        end

    end
end

    %detecting a red circle in captured image
a1 = bwareaopen(I1,30);
se = strel('disk',2);
a1 = imclose(I1,se);
a1=  imfill(I1,'holes');
[B,L1] = bwboundaries(a1,'noholes');
s1=regionprops(L1,'basic');
L2=bwlabel(I1);
s2=regionprops(L2,'basic');
for k= 1:length(B)
    boundary=B{k};
    delta_sq = diff(boundary).^2;
    perimeter = sum(sqrt(sum(delta_sq,2)));
    area = s1(k).Area;
    metric = 4*pi*area/perimeter^2;
    if(metric>=0.94)
            cen = s1(k).Centroid
            x1=cen(:,1)
            y1=cen(:,2)
    end
end
x1=floor(x1);
y1=floor(y1);
cnt1=0;

    %detecting two red regions and their cenroids in captured image
for i=1:numel(s2)
    if(s2(i).Area>50&&s2(i).Area<5000)
        cen1=s2(i).Centroid
        if(cnt1==0)
            x12=cen1(:,1);
            y12=cen1(:,2);
            cnt1=cnt1+1;
        elseif(cnt1==1)
            x13=cen1(:,1);
            y13=cen1(:,2);
            cnt1=cnt1+1;
        else
            break;
        end
    end
end

    %determining which of the 2 centroids belong to the red rectangle
if(abs(x12-x1)<abs(x13-x1))
    x2=x13;
    y2=y13;
else
    x2=x12;
    y2=y12;
end
x2=floor(x2);
y2=floor(y2);

    %detecting a blue circle in captured image
a2 = bwareaopen(I3,30);
se1 = strel('disk',2);
a2 = imclose(I3,se1);
a2=  imfill(I3,'holes');
[B,L3] = bwboundaries(a2,'noholes');
s3=regionprops(L3,'basic');
L4=bwlabel(I3);
s4=regionprops(L4,'basic');
for k= 1:length(B)
    boundary=B{k};
    delta_sq = diff(boundary).^2;
    perimeter = sum(sqrt(sum(delta_sq,2)));
    area = s3(k).Area;
    metric = 4*pi*area/perimeter^2;
    if(metric>=0.94)
        cen2 = s3(k).Centroid
        x3=cen2(:,1)
        y3=cen2(:,2)
    end
end
x3=floor(x3);
y3=floor(y3);
cnt1=0;

    %detecting two blue regions and their cenroids in captured image
for i=1:numel(s4)
    if(s4(i).Area>50&&s4(i).Area<5000)
        cen3=s4(i).Centroid
        if(cnt1==0)
            x31=cen3(:,1);
            y31=cen3(:,2);
            cnt1=cnt1+1;
        elseif(cnt1==1)
            x32=cen3(:,1);
            y32=cen3(:,2);
            cnt1=cnt1+1;
        else
            break;
        end
    end
end

    %determining which of the 2 centroids belong to the blue rectangle
if(abs(x31-x3)<abs(x32-x3))
    x4=x32;
    y4=y32;
else
    x4=x31;
    y4=y31;
end
x4=floor(x4);
y4=floor(y4);


disp(x1);   % x and y coordinates of red circle on robot
disp(y1);

disp(x2);   % x and y coordinates of red square on robot
disp(y2);

disp(x3);   % x and y coordinates of blue circle on robot
disp(y3);

disp(x4);   % x and y coordinates of blue square on robot
disp(y4);


x12=(x1+x2)/2;
y12=(y1+y2)/2;
x34=(x3+x4)/2;
y34=(y3+y4)/2;

disp(x12);  % x and y coordinates of midpoint between red circle(x1,y1) and red square(x2,y2)
disp(y12);

disp(x34);  % x and y coordinates of midpoint between blue circle(x3,y3) and blue square(x4,y4)
disp(y34);


A=[x1,y1]-[x2,y2];  %determining direction vector for red robo's current orientation(where it is currently facing)
ax1=x1-x2;
ay1=y1-y2;

B=[x34,y34]-[x12,y12];  %determining direction vector for red robo's desired orientation(where it should face)
bx1=x34-x12;
by1=y34-y12;

C=[x3,y3]-[x4,y4];  %determining direction vector for blue robo's current orientation(where it is currently facing)
cx1=x3-x4;
cy1=y3-y4;

D=[x12,y12]-[x34,y34];  %determining direction vector for blue robo's desired orientation(where it should face)
dx1=-bx1;
dy1=-by1;

pi=3.14;

theta1=mod(atan2(ax1*by1-bx1*ay1,ax1*bx1+ay1*by1),2*pi)*180/pi; %calculating angle between 2 vectors(robos) in clockwise direction
theta2=mod(atan2(cx1*dy1-dx1*cy1,cx1*dx1+cy1*dy1),2*pi)*180/pi;

disp(theta1);   % angle by which red robot needs to rotate towards the right in order to face the blue robot
disp(theta2);   % angle by which blue robot needs to rotate towards the right in order to face the red robot

x=theta1/100;
x=floor(x);     % hundred's digit of theta1

y=theta1-(x*100);
y=y/10;
y=floor(y);     % ten's digit of theta1

a=theta2/100;
a=floor(a);     % hundred's digit of theta2

b=theta2-(a*100);
b=b/10;
b=floor(b);     % ten's digit of theta2

fprintf(s,'r'); %sending 'r' through zigbee for 'right'
fprintf(z,'r');
pause(1);
if(x~=0)
   fprintf(s,'b');
   pause(1);
   fprintf(s,'%d',x);   %sending hundred's digit of theta1 to red robot
   pause(1);
end

if(a~=0)
   fprintf(z,'b');
   pause(1);
   fprintf(z,'%d',a);   %sending hundred's digit of theta2 to blue robot
   pause(1);
end

fprintf(s,'a'); 
pause(1);
fprintf(s,'%d',y);  %sending ten's digit of theta1 to red robot

fprintf(z,'a');
pause(1);
fprintf(z,'%d',b);  %sending ten's digit of theta2 to blue robot
pause(1);

fprintf(s,'f'); %sending 'f' through zigbee for 'forward'
fprintf(z,'f');

fclose(s);      % CLOSE COM PORT S
delete(s);     % DELETE S
clear s;
    
fclose(z);      % CLOSE COM PORT Z
delete(z);     % DELETE Z
clear z;
end