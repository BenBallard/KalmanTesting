


dataFile = open('LOG','r')
OdomXF = open('OdomX','w')
OdomYF = open('OdomY','w')
OdomTF = open('OdomT','w')
YawTF = open('YawT','w')

OdomXDF = open('OdomXD','w')
OdomYDF = open('OdomYD','w')
OdomTDF = open('OdomTD','w')
YawTDF = open('YawTD','w')


lwsF = open('LeftWheelSpeed','w')
rwsF = open('RightWheelSpeed','w')
LwtF = open('LeftWheelTick','w')
RwtF = open('RightWheelTick','w')


for line in dataFile:
	value = " "
	if "=" in line:
		value = line.split("=")[1].strip() + "\n"
	if "Lwt = " in line:
		LwtF.write(value)
	
	if "Rwt = " in line:
		RwtF.write(value)

	if "OX =" in line:
		OdomXF.write(value)

	if "OY =" in line:
		OdomYF.write(value)
	
	if "OT =" in line:
		OdomTF.write(value)
	
	if "YT =" in line:
		YawTF.write(value)
	
	if "OXD =" in line:
		OdomXDF.write(value)

	if "OYD =" in line:
		OdomYDF.write(value)
	
	if "OTD =" in line:
		OdomTDF.write(value)
	
	if "YTD =" in line:
		YawTDF.write(value)

	if "LWS =" in line:
		lwsF.write(value)
	
	if "RWS =" in line:
		rwsF.write(value)



OdomXF.close()
OdomYF.close()
OdomTF.close()
YawTF.close()

OdomXDF.close()
OdomYDF.close()
OdomTDF.close()
YawTDF.close()


lwsF.close()
rwsF.close()


