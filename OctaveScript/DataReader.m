
% State
%  X
%  Y
%  Theta
%  NOT Currently using velocities
%  X*
%  Y*
%  Theta*

%Initalization
%/home/ben/workspace/NewPIDForward
cd /home/ben/Projects/KalmanTesting/OctaveScript


FolderName = "/home/ben/Projects/KalmanTesting/Backward";
data = LoadData(FolderName,12);
DataLength = size(data{1})(1)

State = zeros(3,1)
PredictedState = zeros(3,1)

time = .02  % = 20 milliseconds
track = 0.5677
LeftWTpM = 0.00086891
RightWTpM = 0.000870814


H = zeros(8,6)
H(1,1)=3



Diff = 0
R=0
Z=0
X=0

LwP=0
RwP=0
LeftPWT = data{11}(1);
RightPWT = data{12}(1);
OdomTheta=0
OdomX=0
OdomY=0
OdomListX = 0 
OdomListY = 0
for x = 2:DataLength 
	odomX = data{1}(x);
	odomY = data{2}(x);
	odomT = data{3}(x);
	yawT = data{4}(x);
	odomXD = data{5}(x);
	odomYD = data{6}(x);
	odomTD = data{7}(x);
	yawTD = data{8}(x);
	LeftWS = data{9}(x);
	RightWS = data{10}(x);
	RightWT = data{12}(x);
	LeftWT = data{11}(x);
	
	%(request - current) .* ( 1-1./exp(x)).+ current
	
	%###################################################################
	%###################################################################
	%This is some shit that i made to figure out where the tire is post  processing to compare agains the estimated veloctiy fo the tires
	LwC(x) = ((odomT*track+2*odomX)/2)*50;
	RwC(x) = (2 * odomX - (odomT*track+2*odomX)/2)*50;
	LwC(x) = ((LeftWT - LeftPWT) * LeftWTpM)/time;
	RwC(x) = ((RightWT - RightPWT) * RightWTpM)/time;
	LeftPWT = LeftWT;
	RightPWT = RightWT;
	%###################################################################
	%###################################################################
	
	
	
	% this is the prediction step That predicts what the current speed of the vechile is
	%This constant descripes the exponential decay.  A larger constant a slower increase to the given speed
	constant = 0.0525;
	%constant = 0.06;
	LwP = (LeftWS-LwP)*(1-1/exp(constant))+LwP;
	RwP = (RightWS-RwP)*(1-1/exp(constant))+RwP;
	
	
	
	
	%This caluclates t the diffence between what really happened and what was predicted.
	LwCD(x) = LwC(x)-LwP;
	RwCD(x) = RwC(x)-RwP;
	
	%LwCD(x) = LeftWS-LwP;
	%RwCD(x) = RightWS-RwP;
	
	RwG(x) = RwP;
	LwG(x) = LwP;
	
	%Run the kalman filter
	
	LastTheta =State(3,1);
	LastX = State(1,1);
	LastY = State(2,1);
	
	PredictedLastTheta = PredictedState(3,1,x-1);
	PredictedLastX = PredictedState(1,1,x-1);
	PredictedLastY = PredictedState(2,1,x-1);	
	
	%Do angle prediction
	dTheta = ((LwP - RwP) * time)/track;
	PredictedAngularVelocity = dTheta/time;
	PredictedTheta = Angle(dTheta + PredictedLastTheta);
	PredictedState(3,1,x) = PredictedTheta;
	PredictedX=0;
	PredictedY=0;
	if ((RightWS-LeftWS) != 0)
		ChangeFactor = (track *(LwP + RwP))/(2*(LwP - RwP));
		PredictedX = PredictedLastX + ChangeFactor*( sin(PredictedTheta) - sin(PredictedLastTheta));
		PredictedY = PredictedLastY+ ChangeFactor *( cos(PredictedTheta) - cos(PredictedLastTheta));
		%Diff = (LastX +RightWS *time*cos(PredictedTheta)) + Diff
		%R = PredictedX + R;
	else
		PredictedX = PredictedLastX + RwP *time*cos(PredictedLastTheta);
		PredictedY = PredictedLastY - RwP*time*sin(PredictedLastTheta);
		%Diff = (LastX +RightWS *time*cos(PredictedTheta)) + Diff
		
		%R = R +(LastX +RightWS *time*cos(PredictedTheta))
	endif
	
	OdomTheta = Angle(OdomTheta + odomT)
 	OdomX = OdomX + cos(OdomTheta)*odomX
 	OdomY = OdomY- sin(OdomTheta)*odomX 
	
	
	OdomListX(x) = OdomX;
	OdomListY(x) = OdomY;
	
	
	
	
	PredictedTheta
	PredictedX
	PredictedY
	PredictedState(1,1,x) = PredictedX;
	PredictedState(2,1,x) = PredictedY;
	
	X = Angle(odomT + X);
	Z=Z+odomX*cos(X);
	
	%plot(x, (request - current) .* ( 1-1./exp(x)).+ current)

	
	
	
	
	
endfor

BadLeft=0;
BadRight=0;
for x = 1:DataLength
	BadRight = BadRight + abs(RwCD(x));
	BadLeft = BadLeft + abs(LwCD(x));
endfor
BadRight;
BadLeft;

HighLimit = 0.5;
LowLimit = -0.5;

LeftLimit = 0
RightLimit = DataLength


figure(1)
subplot (3, 2, 5)
plot(LwCD)
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
subplot (3, 2, 6)
plot(RwCD)
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
subplot (3, 2, 1)
plot(data{9})
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
subplot (3, 2, 2)
plot(data{10})
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
subplot (3, 2, 3)
plot(LwC)
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
subplot (3, 2, 4)
plot(RwC)
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);





figure(2)
clf()
%subplot (3, 2, 5)
%plot(RwCD,'1')
%axis([0,DataLength,LowLimit,HighLimit]);
%subplot (3, 2, 6)
%plot(LwCD,'1')
%axis([0,DataLength,LowLimit,HighLimit]);
subplot (1, 2, 2)
hold on
plot(RwC,'3')
plot(RwG,'1')
plot(RwCD,'4')
axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
hold off 

subplot (1, 2, 1)
hold on
plot(LwC,'3')
plot(LwG,'1')
plot(LwCD,'4')
hold off

axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 3)
%plot(RwG,'4')
%axis([0,DataLength,LowLimit,HighLimit]);
%subplot (3, 2, 4)
%plot(LwG,'4')
%axis([0,DataLength,LowLimit,HighLimit]);


figure(3)
clf()
hold on
%Red
plot(PredictedState(1,1,:),PredictedState(2,1,:),"*1");
%Green
plot(OdomListX(:),OdomListY(:),"*2");
hold off
