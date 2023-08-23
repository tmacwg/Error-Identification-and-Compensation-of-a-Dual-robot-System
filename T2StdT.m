function oT=T2StdT(iT);
iR=iT(1:3,1:3);
rpy=R2RPY(iR);
oR=RPY2R(rpy);
oT=[oR,iT(1:3,4);0,0,0,1;];

