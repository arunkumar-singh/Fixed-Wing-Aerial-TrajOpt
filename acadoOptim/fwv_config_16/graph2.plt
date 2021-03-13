set dummy u, v
set parametric
set isosamples 50, 10
set style data lines
set title "FWV avoiding obstacles" 
set urange [ -1.57080 : 1.57080 ] noreverse nowriteback
set vrange [ 0.00000 : 6.28319 ] noreverse nowriteback
set view equal xyz
#set xrange [-20 : 100]
#set yrange [-20 : 100]
#set zrang [-15 : 100]
set xlabel "x in m"
set ylabel "y in m"
set zlabel "z in m"
#set xyplane relative 0.0

R=10

x1 = 10
x2 = 45
x3 = 50
x4 = 70
x5 = 110
x6 = 15
x7 = 40
x8 = 60
x9 = 100
x10 = 120 
x11 = 80 
x12 = 100
x13 = 0

y1 = 55
y2 = 50
y3=  10
y4 = 70
y5 = 45
y6 = 100
y7 = 110
y8 = 105
y9 =  95
y10 =  80
y11 =  120
y12 = 0
y13 = 20
 
a1 = 5
a2 = 5 
a3 =  10 
a4 = 5 
a5 = 5 
a6 = 12 
a7 = 5 
a8 = 7 
a9 =  5 
a10 =  5 
a11 =  9
a12 = 5
a13 =  5 
	
b1 =5 
b2 = 5  
b3 = 10 
b4 =  5
b5=  5 
b6 = 12
b7 = 5
b8=  7
b9 =   6
b10 =   7
b11 =   9
b12 =    5
b13 =  5
	
c1 = 25 + 10
c2 =   35
c3 =  65
c4 =  25 + 40 + 20
c5 = 25 
c6 = 50 
c7 = 25 + 30
c8 = 35 
c9 = 25 +25
c10 =  25+40
c11 =   75
c12 =   25+15
c13 =  45
z = 0

set terminal qt 0 
splot x1 + a1*cos(u)*sin(v),y1 + b1*sin(u)*sin(v) ,z + c1*cos(v) notitle ,\
	 x2 + a2*cos(u)*cos(v),y2 + b2*cos(u)*sin(v),z + c2*sin(u) notitle ,\
	 x3 + a3*cos(u)*cos(v),y3 + b3*cos(u)*sin(v),z + c3*sin(u) notitle ,\
	  x4 + a4*cos(u)*cos(v),y4 + b4*cos(u)*sin(v),z + c4*sin(u) notitle,\
	 x5 + a5*cos(u)*cos(v),y5 + b5*cos(u)*sin(v),z + c5*sin(u) notitle ,\
	x6 + a6*cos(u)*cos(v),y6 + b6*cos(u)*sin(v),z + c6*sin(u) notitle ,\
	x7 + a7*cos(u)*cos(v),y7 + b7*cos(u)*sin(v),z + c7*sin(u) notitle ,\
	x8 + a8*cos(u)*cos(v),y8 + b8*cos(u)*sin(v),z + c8*sin(u) notitle ,\
	x9 + a9*cos(u)*cos(v),y9 + b9*cos(u)*sin(v),z + c9*sin(u) notitle ,\
	x10 + a10*cos(u)*cos(v),y10 + b10*cos(u)*sin(v),z + c10*sin(u) notitle ,\
	x11 + a11*cos(u)*cos(v),y11 + b11*cos(u)*sin(v),z + c11*sin(u) notitle ,\
	x11 + a12*cos(u)*cos(v),y12 + b12*cos(u)*sin(v),z + c12*sin(u) notitle ,\
	x13 + a13*cos(u)*cos(v),y13 + b13*cos(u)*sin(v),z + c13*sin(u) notitle ,\
	"datags.txt" title "start-goal" with points,\
	"data.txt" title "One loop Trajectory with Obs"

pause -1
set terminal qt 1
plot "phi.txt"

pause -1








set terminal qt 1 
plot "phi.dat"
pause -1
