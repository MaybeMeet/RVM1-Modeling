function traj_plot()
x=[13 14 6  2  5 20 42 49 39 25 22 30];
y=[48 31 7 13 19 12 16 39 61 66 40  2];
n=length(x);
t=1:n;
tt=linspace(t(1),t(n),50);
xx=spline(t,x,tt)/40;
yy=spline(t,y,tt)/40;
zz = tt -7 ;
%z=zeros(size(xx));
plot3(xx'+ 1,z',yy'+3,'b')
%points =[ xx',z',yy']
%axis('equal'),axis('off')
end
