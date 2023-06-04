clc
clear
close all

canopy.EdgeLen = 60;%cm
canopy.InitialLen = 50;
canopy.CenterX = 0;%cm
canopy.CenterY = 0;%cm
canopy.CenterZ = 37;%cm
canopy.AngleX = 0;%degree
canopy.AngleY = 0;%degree

light.R = 200;%cm
light.Elevation = 64;%degree
light.Azimuth = 0;%degree

%Tentin yerden yükseklikleri
% SW : 36.5, NW: 37.5, NE: 39.8, SE: 38 cm


% [Shadow,Pole]=CalcShadow(canopy,light)
% CalcArea(Shadow);
% PlotGround(canopy,Shadow);
% figure
% Plot3DSpace(Pole,canopy,Shadow,light);
% CalcIntersectionArea(canopy,Shadow)
% UnionArea = canopy.EdgeLen^2+CalcArea(Shadow)-CalcIntersectionArea(canopy,Shadow)
% Pole

index=1;
opt=0;
for xchange=-15:0.2:15  % -x: west, +x: east
    canopy.CenterX = xchange; 
    for ychange=-20:0.2:20 % -y: south, +y: north
        canopy.CenterY = ychange;
        [Shadow,Pole]=CalcShadow(canopy,light);
        temp = CalcOpt(canopy, Shadow);
        if(temp>opt)
            opt=temp
            CanopyValuesMax = canopy;
            PoleValuesMax = Pole;
            ShadowValuesMax = Shadow;
        end
%         opt(index) = CalcOpt(canopy, Shadow);
%         CanopyValues(index) = canopy;
%         PoleValues(index) =  Pole;
%         ShadowValues(index) = Shadow;
        index = index+1;
    end
end

% % index=1;
% % opt=0;
% % for xangle=-5:1:5
% %     canopy.AngleX = xangle;
% %     for yangle=-5:1:5
% %         canopy.AngleY = yangle;
% %         for xchange=-20:2:20
% %             canopy.CenterX = xchange;
% %             for ychange=-20:2:20
% %                 canopy.CenterY = ychange;
% %                 [Shadow,Pole]=CalcShadow(canopy,light);
% %                 temp = CalcOpt(canopy, Shadow);
% %                 if(temp>opt)
% %                     opt=temp
% %                     CanopyValuesMax = canopy;
% %                     PoleValuesMax = Pole;
% %                     ShadowValuesMax = Shadow;
% %                 end
% %                 %CanopyValues(index) = canopy;
% %                 %PoleValues(index) =  Pole;
% %                 %ShadowValues(index) = Shadow;
% %                 index = index+1;
% %             end
% %         end
% %     end
% % end

max(opt)
Plot3DSpace(PoleValuesMax,CanopyValuesMax,ShadowValuesMax,light);
figure
PlotGround(CanopyValuesMax,ShadowValuesMax);

function [YourShadow, PoleEnds] = CalcShadow(CanopyValues,LightValues)
%assume 0 0 0 is ground, where we saw shadow

%transform coordinates to make easier calculation
[MyLight.x,MyLight.y,MyLight.z] = sph2cart(deg2rad(LightValues.Azimuth),deg2rad(LightValues.Elevation),LightValues.R);

%find edge coordinates of canopy
% C3---E4----C4
% ||||||||||||
% E1|||||||||E3
% ||||||||||||
% C1---E2----C2

Edge1Vect.x = 0;
Edge1Vect.y = - (CanopyValues.EdgeLen/2);
Edge1Vect.z = 0;
[Edge1Vect.x,Edge1Vect.y,Edge1Vect.z] = RotateX(Edge1Vect.x,Edge1Vect.y,Edge1Vect.z,CanopyValues.AngleX);
[Edge1Vect.x,Edge1Vect.y,Edge1Vect.z] = RotateY(Edge1Vect.x,Edge1Vect.y,Edge1Vect.z,CanopyValues.AngleY);
Edge1Vect.len = sqrt(Edge1Vect.x^2+Edge1Vect.y^2+Edge1Vect.z^2)/(25);

Edge2Vect.x = (CanopyValues.EdgeLen/2);
Edge2Vect.y = 0;
Edge2Vect.z = 0;
[Edge2Vect.x,Edge2Vect.y,Edge2Vect.z] = RotateX(Edge2Vect.x,Edge2Vect.y,Edge2Vect.z,CanopyValues.AngleX);
[Edge2Vect.x,Edge2Vect.y,Edge2Vect.z] = RotateY(Edge2Vect.x,Edge2Vect.y,Edge2Vect.z,CanopyValues.AngleY);
Edge2Vect.len = sqrt(Edge2Vect.x^2+Edge2Vect.y^2+Edge2Vect.z^2)/(25);

Edge3Vect.x = 0;
Edge3Vect.y = (CanopyValues.EdgeLen/2);
Edge3Vect.z = 0;
[Edge3Vect.x,Edge3Vect.y,Edge3Vect.z] = RotateX(Edge3Vect.x,Edge3Vect.y,Edge3Vect.z,CanopyValues.AngleX);
[Edge3Vect.x,Edge3Vect.y,Edge3Vect.z] = RotateY(Edge3Vect.x,Edge3Vect.y,Edge3Vect.z,CanopyValues.AngleY);
Edge3Vect.len = sqrt(Edge3Vect.x^2+Edge3Vect.y^2+Edge3Vect.z^2)/(25);

Edge4Vect.x =  - (CanopyValues.EdgeLen/2);
Edge4Vect.y = 0;
Edge4Vect.z = 0;
[Edge4Vect.x,Edge4Vect.y,Edge4Vect.z] = RotateX(Edge4Vect.x,Edge4Vect.y,Edge4Vect.z,CanopyValues.AngleX);
[Edge4Vect.x,Edge4Vect.y,Edge4Vect.z] = RotateY(Edge4Vect.x,Edge4Vect.y,Edge4Vect.z,CanopyValues.AngleY);
Edge4Vect.len = sqrt(Edge4Vect.x^2+Edge4Vect.y^2+Edge4Vect.z^2)/(25);

%Sum of Edge1 vector and Edge2 vector will give C1 vector. other can be
%derived with same logic

Corner1Vect.x = Edge1Vect.x + Edge2Vect.x;
Corner1Vect.y = Edge1Vect.y + Edge2Vect.y;
Corner1Vect.z = Edge1Vect.z + Edge2Vect.z;
Corner1Vect.len = sqrt(Corner1Vect.x^2+Corner1Vect.y^2+Corner1Vect.z^2)/(25*sqrt(2));

Corner2Vect.x = Edge2Vect.x + Edge3Vect.x;
Corner2Vect.y = Edge2Vect.y + Edge3Vect.y;
Corner2Vect.z = Edge2Vect.z + Edge3Vect.z;
Corner2Vect.len = sqrt(Corner2Vect.x^2+Corner2Vect.y^2+Corner2Vect.z^2)/(25*sqrt(2));

Corner3Vect.x = Edge1Vect.x + Edge4Vect.x;
Corner3Vect.y = Edge1Vect.y + Edge4Vect.y;
Corner3Vect.z = Edge1Vect.z + Edge4Vect.z;
Corner3Vect.len = sqrt(Corner3Vect.x^2+Corner3Vect.y^2+Corner3Vect.z^2)/(25*sqrt(2));

Corner4Vect.x = Edge3Vect.x + Edge4Vect.x;
Corner4Vect.y = Edge3Vect.y + Edge4Vect.y;
Corner4Vect.z = Edge3Vect.z + Edge4Vect.z;
Corner4Vect.len = sqrt(Corner4Vect.x^2+Corner4Vect.y^2+Corner4Vect.z^2)/(25*sqrt(2));

%add vectors to center and get the corner points.

Corner1.x = Corner1Vect.x + CanopyValues.CenterX;
Corner1.y = Corner1Vect.y + CanopyValues.CenterY;
Corner1.z = Corner1Vect.z + CanopyValues.CenterZ;


Corner2.x = Corner2Vect.x + CanopyValues.CenterX;
Corner2.y = Corner2Vect.y + CanopyValues.CenterY;
Corner2.z = Corner2Vect.z + CanopyValues.CenterZ;

Corner3.x = Corner3Vect.x + CanopyValues.CenterX;
Corner3.y = Corner3Vect.y + CanopyValues.CenterY;
Corner3.z = Corner3Vect.z + CanopyValues.CenterZ;

Corner4.x = Corner4Vect.x + CanopyValues.CenterX;
Corner4.y = Corner4Vect.y + CanopyValues.CenterY;
Corner4.z = Corner4Vect.z + CanopyValues.CenterZ;

%we know all the corner points.
PoleEnds.C1 = [Corner1.x,Corner1.y,Corner1.z];
PoleEnds.C2 = [Corner2.x,Corner2.y,Corner2.z];
PoleEnds.C3 = [Corner3.x,Corner3.y,Corner3.z];
PoleEnds.C4 = [Corner4.x,Corner4.y,Corner4.z];

%using parametric equation for line equation in 3D space (solve for z=0) i
%will find the x y points of corners of the shadow.
%assume v1=(a,b,c) v2=(q,w,e) vector is <q-a,w-b,e-c>
% x = a + (q-a) * t
% y = b + (w-b) * t
% z = c + (e-c) * t
% for us z=0, then t = c/(c-e) we can find x and y now.

%start with corner1 always assuming v2 is light, v1 is the corner

temp = (Corner1.z)/(Corner1.z - MyLight.z);
ShadowCorner1.x = Corner1.x + (MyLight.x - Corner1.x)*temp;
ShadowCorner1.y = Corner1.y + (MyLight.y - Corner1.y)*temp;
ShadowCorner1.z = 0;

temp = (Corner2.z)/(Corner2.z - MyLight.z);
ShadowCorner2.x = Corner2.x + (MyLight.x - Corner2.x)*temp;
ShadowCorner2.y = Corner2.y + (MyLight.y - Corner2.y)*temp;
ShadowCorner2.z = 0;

temp = (Corner3.z)/(Corner3.z - MyLight.z);
ShadowCorner3.x = Corner3.x + (MyLight.x - Corner3.x)*temp;
ShadowCorner3.y = Corner3.y + (MyLight.y - Corner3.y)*temp;
ShadowCorner3.z = 0;

temp = (Corner4.z)/(Corner4.z - MyLight.z);
ShadowCorner4.x = Corner4.x + (MyLight.x - Corner4.x)*temp;
ShadowCorner4.y = Corner4.y + (MyLight.y - Corner4.y)*temp;
ShadowCorner4.z = 0;

YourShadow.C1 = [ShadowCorner1.x,ShadowCorner1.y,ShadowCorner1.z];
YourShadow.C2 = [ShadowCorner2.x,ShadowCorner2.y,ShadowCorner2.z];
YourShadow.C3 = [ShadowCorner3.x,ShadowCorner3.y,ShadowCorner3.z];
YourShadow.C4 = [ShadowCorner4.x,ShadowCorner4.y,ShadowCorner4.z];
YourShadow.Center = [0.25*(ShadowCorner1.x+ShadowCorner2.x+ShadowCorner3.x+ShadowCorner4.x) 0.25*(ShadowCorner1.y+ShadowCorner2.y+ShadowCorner3.y+ShadowCorner4.y) 0];

end
function [x,y,z] = RotateY(xr,yr,zr,a)
a=-a;
temp = [1 0 0;
    0 cosd(a) -sind(a);
    0 sind(a) cosd(a)]*[xr;yr;zr];
x = temp(1);
y = temp(2);
z = temp(3);
end
function [x,y,z] = RotateX(xr,yr,zr,a)
temp = [cosd(a) 0 sind(a);
    0 1 0;
    -sind(a) 0 cosd(a)]*[xr;yr;zr];
x = temp(1);
y = temp(2);
z = temp(3);
end
function Area = CalcArea(ShadowValues)
% % % C3---E4----C4
% % % ||||||||||||
% % % E1|||||||||E3
% % % ||||||||||||
% % % C1---E2----C2
% %
% % %Divide it into two triangles (C1 C2 C4) and (C1 C3 C4)
% % AreaOfTriangle1 = abs(0.5 * det([ShadowValues.C1(1) ShadowValues.C1(2) 1;ShadowValues.C2(1) ShadowValues.C2(2) 1;ShadowValues.C4(1) ShadowValues.C4(2) 1;]));
% % AreaOfTriangle2 = abs(0.5 * det([ShadowValues.C1(1) ShadowValues.C1(2) 1;ShadowValues.C3(1) ShadowValues.C3(2) 1;ShadowValues.C4(1) ShadowValues.C4(2) 1;]));
% % area = AreaOfTriangle1+AreaOfTriangle2;
x_shadow = [ShadowValues.C1(1) ShadowValues.C2(1) ShadowValues.C4(1) ShadowValues.C3(1)];
y_shadow = [ShadowValues.C1(2) ShadowValues.C2(2) ShadowValues.C4(2) ShadowValues.C3(2)];
Shadow = polyshape(x_shadow,y_shadow);
Area = area(Shadow);
end
function [ret] = PlotGround(CanopyValues, ShadowValues)
%calculate the initial shadow, based on canopy edge len, located at x=0 y=0
initialShadowEdge = CanopyValues.InitialLen;
x_initial = [initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2 -initialShadowEdge/2];
y_initial = [-initialShadowEdge/2 initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2];
%shadow values
x_shadow = [ShadowValues.C1(1) ShadowValues.C2(1) ShadowValues.C4(1) ShadowValues.C3(1)];
y_shadow = [ShadowValues.C1(2) ShadowValues.C2(2) ShadowValues.C4(2) ShadowValues.C3(2)];
title("red initial, black shadow")
patch(x_shadow, y_shadow, 'black')
patch(x_initial, y_initial, 'red')
ret = 1;
end
function [ret] = Plot3DSpace(PoleEnds, CanopyValues, ShadowValues, LightValues)
[MyLight.x,MyLight.y,MyLight.z] = sph2cart(deg2rad(LightValues.Azimuth),deg2rad(LightValues.Elevation),LightValues.R);

initialShadowEdge = CanopyValues.InitialLen;
x_initial = [initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2 -initialShadowEdge/2];
y_initial = [-initialShadowEdge/2 initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2];
z_initial = [0 0 0 0];

x_shadow = [ShadowValues.C1(1) ShadowValues.C2(1) ShadowValues.C4(1) ShadowValues.C3(1)];
y_shadow = [ShadowValues.C1(2) ShadowValues.C2(2) ShadowValues.C4(2) ShadowValues.C3(2)];
z_shadow = [0 0 0 0];

x_poles = [PoleEnds.C1(1) PoleEnds.C2(1) PoleEnds.C4(1) PoleEnds.C3(1)];
y_poles = [PoleEnds.C1(2) PoleEnds.C2(2) PoleEnds.C4(2) PoleEnds.C3(2)];
z_poles = [PoleEnds.C1(3) PoleEnds.C2(3) PoleEnds.C4(3) PoleEnds.C3(3)];

fill3(x_shadow,y_shadow,z_shadow,'black')
hold on
fill3(x_poles,y_poles,z_poles,'blue')
fill3(x_initial,y_initial,z_initial,'red')
plot3(MyLight.x,MyLight.y,MyLight.z,"hexagram",'Color','green')

ret = 1;
end
function IntersectionArea = CalcIntersectionArea(CanopyValues, ShadowValues)

initialShadowEdge = CanopyValues.InitialLen;
x_initial = [initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2 -initialShadowEdge/2];
y_initial = [-initialShadowEdge/2 initialShadowEdge/2 initialShadowEdge/2 -initialShadowEdge/2];
initialShadow = polyshape(x_initial,y_initial);

x_shadow = [ShadowValues.C1(1) ShadowValues.C2(1) ShadowValues.C4(1) ShadowValues.C3(1)];
y_shadow = [ShadowValues.C1(2) ShadowValues.C2(2) ShadowValues.C4(2) ShadowValues.C3(2)];
Shadow = polyshape(x_shadow,y_shadow);

intersection = intersect(initialShadow,Shadow);

IntersectionArea = area(intersection);

end
function OptVal = CalcOpt(CanopyValues,Shadow)
initialArea = CanopyValues.InitialLen^2;
shadowArea = CalcArea(Shadow);
intersectionArea = CalcIntersectionArea(CanopyValues,Shadow);
UnionArea = initialArea+shadowArea-intersectionArea;
OptVal = intersectionArea/UnionArea;
end
