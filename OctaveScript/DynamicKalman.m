
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

FolderName = "/home/ben/Projects/KalmanTesting/Forward";
data = LoadData(FolderName,10);
DataLength = size(data{1})(1)

state = zeros(3,1);
PredictedState = zeros(3,1);

time = .02  % = 20 milliseconds
track = 0.5677

H = zeros(4,3);
H(1,1)=1;
H(2,2)=1;
H(3,3)=1;
H(4,3)=1;

G=zeros(3,3);
G(1,1)=1;
G(2,2)=1;
G(3,3)=1;

P=eye(3).*.001





Diff = 0
R=0
Z=0
X=0


%Assume that the system starts up stoped 
%Left Wheel speed Predicted
LwP=0
%Right Wheel Speed Predicted
RwP=0


MODELX = 0;
MODELY =0;

MODELTHETA=0;
OdomListX = 0
StateX=0
StateY=0
OdomListY=0
OdomTheta=0
OdomX=0
OdomY=0
YawTheta=0
OdomListX = 0 
OdomListY = 0
ModelX=0;
ModelY=0;
modelX=0;
modelY=0;
modelTheta=0;
%DataLength=3
for x = 2:DataLength
%for x = 400:500
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
	
	
	
	% this is the prediction step That predicts what the current speed of the vechile is
	%This constant descripes the exponential decay.  A larger constant a slower increase to the given speed
	constant = 0.0525;
	
	LwP = (LeftWS-LwP)*(1-1/exp(constant))+LwP;
	RwP = (RightWS-RwP)*(1-1/exp(constant))+RwP;
	

	
	
	
	%Run the kalman filter
	
	LastTheta =state(3,1);
	LastX = state(1,1);
	LastY = state(2,1);
	%LastY=modelY;
	%LastX=modelX;
	%LastTheta=modelTheta;
%	LastTheta = PredictedState(3,1,x-1);
%	LastX = PredictedState(1,1,x-1);
%	LastY = PredictedState(2,1,x-1);	
	
	%Do angle prediction
	dTheta = ((LwP - RwP) * time)/track;
	modelTheta = dTheta + LastTheta;
	LastModelTheta = MODELTHETA;
	MODELTHETA=dTheta+MODELTHETA;
	if ((RwP-LwP) != 0)
		
		ChangeFactor = (track *(LwP + RwP))/(2*(LwP - RwP));
		modelX = LastX + ChangeFactor*( sin(modelTheta) - sin(LastTheta));
		modelY = LastY+ ChangeFactor *( cos(modelTheta) - cos(LastTheta));
		MODELX(x) = MODELX(x-1)+ ChangeFactor*( sin(MODELTHETA) - sin(LastModelTheta));
		MODELY(x) = MODELY(x-1)+ ChangeFactor*( cos(MODELTHETA) - cos(LastModelTheta));
		%THIS HEADING MIGHT BE WRONG
		G(1,3)=ChangeFactor * (-cos(LastTheta)+cos(modelTheta));
		G(2,3) = ChangeFactor * (-sin(LastTheta)+sin(modelTheta));
		%Diff = (LastX +RightWS *time*cos(PredictedTheta)) + Diff
		%R = PredictedX + R;
	else
		modelX = LastX + RwP *time*cos(LastTheta);
		modelY = LastY - RwP*time*sin(LastTheta);
		MODELX(x) = MODELX(x-1)+ RwP*time*cos(MODELTHETA);
		MODELY(x) = MODELY(x-1)- RwP*time*sin(MODELTHETA);
		
		
		%THIS HEADING MIGHT BE WRONG
		G(1,3)= -1*RwP*time*sin(LastTheta);
		G(2,3) = RwP*time*cos(LastTheta);
		%Diff = (LastX +RightWS *time*cos(PredictedTheta)) + Diff
		%R = R +(LastX +RightWS *time*cos(PredictedTheta))
	endif
	%MODELX(x)=LastX;
	%MODELY(x) = LastY;
	
	
	
	
	
	%Note at this point modelTheta may not be witht the appropriate angles
	%Note this  model solves the Predict the next state
	model = zeros(3,1);
	model(1,1) = modelX;
	model(2,1) = modelY;
	model(3,1) = modelTheta;
	state(1,1) = modelX;
	state(2,1) = modelY;
	state(3,1) = modelTheta;
	
	
	
	
	%Basic odometry Stuff to comapre the kalman filter against.
	OdomTheta = Angle(OdomTheta + odomT);
 	OdomX = OdomX+ cos(OdomTheta)*odomX;
 	OdomY = OdomY- sin(OdomTheta)*odomX ;
	OdomListX(x) = OdomX;
	OdomListY(x) = OdomY;
	
	%yaw
	YawTheta = YawTheta + yawT;
	
	
	OdomTheta = Angle(LastTheta + odomT);
 	RodomX = cos(OdomTheta)*odomX;
 	RodomY = sin(OdomTheta)*odomX ;
	
	
	
	
	
	H = zeros(4,3);
	H(1,1)=1;
	H(2,2)=1;
	H(3,3)=1;
	H(4,3)=1;
	
	%Placed the sensed values into an array
	Zsensed(1,1)= RodomX+LastX;
	Zsensed(2,1)= -1*RodomY+LastY;
	Zsensed(3,1)= LastTheta + odomT;
	Zsensed(4,1)=  yawT+LastTheta;
	MODELX(x) = RodomX+LastX;
	MODELY(x)  = RodomY + LastY;
	
	%Zsensed(1,1)=OdomX;
	%Zsensed(2,1)= OdomY;
	%Zsensed(3,1)= OdomTheta;
	%Zsensed(4,1)=  YawTheta;
		
	
	%This is the Predict the estimated covariance
	modelSTD=eye(3).*.5;
	P=G*P*G'+modelSTD
	Y = Zsensed- H*state;
	sensorSTD = eye(4).*.5
	S = H*P*H'+sensorSTD
	K=P*H'*inv(S)
	state=state+K*Y
	P=(eye(3)-K*H)*P
	

	rP = 4;
	rPMO = 1 - rP; 
	oldValue = RodomX;
	RodomX = oldValue*rP *(rand(1)-.5) + oldValue;
	oldValue = RodomY;
	RodomY = oldValue*rP *(rand(1)-.5) +oldValue;
	oldValue = odomT;
	odomT= oldValue*rP *(rand(1)-.5) +oldValue;
	oldValue = yawT;
	yawT= oldValue*rP *(rand(1)-.5) +oldValue;
	Zsensed = zeros(1,1);
	Zsensed(1,1)= LastTheta + odomT;

	H = zeros(1,3);
	H(1,3)=1;

	Y = Zsensed- H*state;
	sensorSTD = eye(1).*.02.* (rand(1)*1-.5);
	
	S = H*P*H'+sensorSTD
	K=P*H'*inv(S)
	state=state+K*Y
	P=(eye(3)-K*H)*P
	
	
	
	
	
	
	
	
	ARG(x)=P(2,2);
	
	%Does this need to be done
	%state(3,1)=Angle(state(3,1));
	
	
	
	
	state
%	PredictedTheta;
%	PredictedX
%	PredictedY
%	PredictedState(1,1,x) = PredictedX;
%	PredictedState(2,1,x) = PredictedY;
%	X = Angle(odomT + X);
%	Z=Z+odomX*cos(X);



	StateX(x)= state(1,1);
	StateY(x)=state(2,1);
	

	
	
endfor
OdomX
OdomY

HighLimit = 1
LowLimit = -1;

LeftLimit = 0
RightLimit = DataLength


%figure(1)
%subplot (3, 2, 5)

%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 6)

%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 1)
%plot(data{9})
%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 2)
%plot(data{10})
%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 3)

%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%subplot (3, 2, 4)

%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);





figure(2)
clf()
plot(ARG(:))
%subplot (3, 2, 5)
%plot(RwCD,'1')
axis([0,DataLength,LowLimit,HighLimit]);
%subplot (3, 2, 6)
%plot(LwCD,'1')
%axis([0,DataLength,LowLimit,HighLimit]);
%subplot (1, 2, 2)
%hold on
%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
%hold off 
%subplot (1, 2, 1)
%hold on
%hold off
%axis([LeftLimit,RightLimit,LowLimit,HighLimit]);
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
plot(StateX(:),StateY(:),"*1");
%Green
plot(OdomListX(:),OdomListY(:),"*2");

plot(MODELX(:),MODELY(:),"*3");
hold off