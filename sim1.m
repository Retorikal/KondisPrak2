%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PID Control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc, clear all, warning off
SP=55;
Gain=1.0308;
tau=0.0728;
dt=0.02;

ft = 10; 
Kp = 1.059;
Ti = 0.160;
Td = 0.004;
P = 1; I = 1; D = 1;
df = round(ft/dt); 
dlay = 0.08/dt;

h1(1:df) = 0;  h2(1:df) = 0; h3(1:df) = 0;h4(1:df) = 0; Fi(1:df)=0;
SIn(1:df) = 0; e_L = 0;  
SIn3(1:df) = 0; e_LN3 = 0; e_LP3 = 0;
SIn4(1:df) = 0; SLIn4 = 0; e_LN4 = 0; e_LP4 = 0;
MV2(1:df) = 0; MV3(1:df) = 0; MV4(1:df) = 0;

t(1:1:dlay+1) = dt*[0:1:dlay];
t2(1:1:dlay+1) = t(1:1:dlay+1);
t3(1:1:dlay+1) = t(1:1:dlay+1); 
t4(1:1:dlay+1) = t(1:1:dlay+1);


for i = dlay+1:1:df+1
	Fi(i) = Gain*SP; 
	dh1dt = (Fi(i-dlay) - h1(i))/tau;
	h1(i+1)  = h1(i) + dh1dt*dt;
	t(i+1)	= t(i) + dt;
	
	%___________Original PID Control__________
	
	e_B	= SP - h2(i);

	KKp	= P * Kp * e_B;

	if i == 1
		SIn(i) = (e_B+e_L)/2*dt;
	else
		SIn(i) = SIn(i-1) + (e_B+e_L)/2*dt;
	end
	
	KIn	= I * Kp * SIn(i) / Ti;

	DTd	= (e_B-e_L)/dt;
	KDr	= D * Kp * DTd * Td;

	MV2(i)	= KKp + KIn + KDr;
	e_L = e_B;
	
	dh2dt = (MV2(i-dlay)-h2(i))/tau;
	h2(i+1)  = h2(i) + dh2dt*dt;
	t2(i+1)  = t2(i) + dt;

	
	%%____Mikrocontroller nonnegative computation_________________________
	
	if SP >= h3(i)
		e_BP3 = SP - h3(i);
		e_BN3 = 0;
	else
		e_BP3 = 0;
		e_BN3 = h3(i) - SP;
	end
	
	KKpP3 = P * Kp * e_BP3;
	KKpN3 = P * Kp * e_BN3;
	
	SPIn3 = 0; SNIn3 = 0;
	if and(e_BN3 == 0,e_LN3==0)
		SPIn3 = (e_BP3+e_LP3)/2*dt;
	elseif and(e_BP3 == 0,e_LP3==0)
		SNIn3 = (e_BN3+e_LN3)/2*dt;
	elseif and(e_BN3 > 0,e_LP3 > 0)
		if e_BN3 >= e_LP3
			SNIn3 = (e_BN3-e_LP3)/(2/dt);
		else
			SPIn3 = (e_LP3-e_BN3)/(2/dt);
		end
	elseif and(e_BP3 > 0,e_LN3 > 0)
		if e_BP3 >= e_LN3
			SPIn3 = (e_BP3-e_LN3)/(2/dt);
		else
			SNIn3 = (e_LN3-e_BP3)/(2/dt);
		end
	end

	if i == 1
		SIn3(i) = SPIn3 - SNIn3;
	else
		SIn3(i) = SIn3(i-1) + SPIn3 - SNIn3;
	end
	
	KIn3	= I * Kp * SIn3(i) / Ti;
	
	SD = 0;
	if and(e_BP3 == 0,e_LN3==0)
		DTd3 = (e_LP3+e_BN3)/dt;
		SD = 1;
	elseif and(e_BN3 == 0,e_LP3==0)
		DTd3 = (e_BP3+e_LN3)/dt;
	elseif and(e_BP3 > 0,e_LP3 > 0)
		if e_BP3 >= e_LP3
			DTd3 = (e_BP3-e_LP3)/dt;
		else
			DTd3 = (e_LP3-e_BP3)/dt;
			SD = 1;
		end
	elseif and(e_BN3 > 0,e_LN3 > 0)
		if e_BN3 >= e_LN3
			DTd3 = (e_BN3-e_LN3)/dt;
			SD = 1;
		else
			DTd3 = (e_LN3-e_BN3)/dt;
		end
	end
	
	KDr3	= D * Kp * DTd3 * Td;

	if SD == 0
		MV3(i)	= KKpP3 - KKpN3 + KIn3 + KDr3;
	else
		MV3(i)	= KKpP3 - KKpN3 + KIn3 - KDr3;
	end
	e_LP3 = e_BP3;
	e_LN3 = e_BN3;
	
	dh3dt = (MV3(i-dlay)-h3(i))/tau;
	h3(i+1)  = h3(i) + dh3dt*dt;
	t3(i+1)  = t3(i) + dt;

	%%____Application on Mikrocontroller _________________________
	if SP >= h4(i)
		e_BP4 = SP - h4(i);  
		e_BN4 = 0;
	else
		e_BP4 = 0;
		e_BN4 = h4(i) - SP;
	end

	%% P
	KKpP4 = ( e_BP4 * Kp );
	KKpN4 = ( e_BN4 * Kp );

	%% I
	SPIn4 = 0; SNIn4 = 0;		
	acc = 2/dt;
	if and( e_BN4 == 0, e_LN4 == 0 )
		acc1 = e_BP4 + e_LP4;				
		SPIn4 = acc1 / acc;					
	elseif and( e_BP4 == 0, e_LP4 == 0 )
		acc1 = e_BN4 + e_LN4;				
		SNIn4 = acc1 / acc;
	elseif and( e_LP4 > 0, e_BN4 > 0 )	
		if e_BN4 >= e_LP4
			acc1 = e_BN4 - e_LP4;
			SNIn4 = acc1 / acc;
		else
			acc1 = e_LP4 - e_BN4;
			SPIn4 = acc1 / acc;
		end
	elseif and( e_BP4 > 0, e_LN4 > 0 )	
		if e_LN4 >= e_BP4
			acc1 = e_LN4 - e_BP4;
			SPIn4 = acc1 / acc;
		else
			acc1 = e_BP4 - e_LN4;
			SNIn4 = acc1 / acc;
		end
	end

	if ( i == 1 )
		SIn4(i) = SPIn4 - SNIn4;
	else
		SIn4(i) = SLIn4 + SPIn4 - SNIn4;
	end

	SLIn4 = SIn4(i);
	acc1 = Kp * SIn4(i);
	acc2 = acc1 / Ti;
	KIn4 = acc2 * I;
	%% D	
	SD = 0;
	if and(e_BP4 == 0,e_LN4==0)
		acc1 = e_LP4 + e_BN4;
		SD = 1;
	elseif and(e_BN4 == 0,e_LP4==0)
		acc1 = e_BP4 + e_LN4;
	elseif and(e_BP4 > 0,e_LP4 > 0)
		if e_BP4 >= e_LP4
			acc1 = e_BP4 - e_LP4;
		else
			acc1 = e_LP4 - e_BP4;
			SD = 1;
		end
	elseif and(e_BN4 > 0,e_LN4 > 0)
		if e_BN4 >= e_LN4
			acc1 = e_BN4 - e_LN4;
			SD = 1;
		else
			acc1 = e_LN4 - e_BN4;
		end
	end
	DTd4 = acc1 / dt;

	acc1 = Kp * DTd4;
	acc2 = D * Td;
	KDr4 = acc1 * acc2;

	if SD == 0
		MV4(i) = KKpP4 - KKpN4 + KIn4 + KDr4;
	else
		MV4(i) = KKpP4 - KKpN4 + KIn4 - KDr4;
	end

	if MV4(i) >= 255
		MV4(i) = 255;
	elseif MV4(i) <= 0
		MV4(i) = 0; 
	end

	disp([e_BP3 e_LP3 SPIn3 e_BP4 e_LP4 SPIn4 ]);
	
	e_LP4 = e_BP4;	
	e_LN4 = e_BN4;	


	dh4dt = (MV4(i-dlay)-h4(i))/tau;
	h4(i+1)  = h4(i) + dh4dt*dt;
	t4(i+1)  = t4(i) + dt;

end

figure(1)
axis([0 ft min([h1 h2 h3 h4]) max([h1 h2 h3 h4])]);
stairs(t,h1,'r');  hold on; %red
stairs(t2,h2,'b'); %blue
stairs(t3,h3,'k'); %black
%stairs(t4,h4,'m'); %magenta
hold off;
title(['Kecepatan pada SP = ' num2str(SP) ', Kp = ' num2str(Kp) ...
	', Ti = ' num2str(Ti) ', Td = ' num2str(Td) ', dt = ' num2str(dt)])

figure(2)
stairs(t2(1:end-1),MV2,'b'); hold on;
stairs(t3(1:end-1),MV3,'k'); 
%stairs(t4(1:end-1),MV4,'m'); 
hold off;
title(['MV pada SP = ' num2str(SP) ', Kp = ' num2str(Kp) ...
	', Ti = ' num2str(Ti) ', Td = ' num2str(Td) ', dt = ' num2str(dt)])


figure(3)
stairs(t(1:end-1),SIn,'b'); hold on;
stairs(t3(1:end-1),SIn3,'k'); 
%stairs(t4(1:end-1),SIn4,'m'); 
hold off;
title(['Akumulasi error pada SP = ' num2str(SP) ', Kp = ' num2str(Kp) ...
	', Ti = ' num2str(Ti) ', Td = ' num2str(Td) ', dt = ' num2str(dt)])

