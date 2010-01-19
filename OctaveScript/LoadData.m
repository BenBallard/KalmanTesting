function data = LoadData(Folder)
  if (nargin != 1)
    error ("usage: LoadData(Folder)");
  endif
printf("%s",Folder)
filelist{1} = sprintf("%s/%s",Folder,"OdomX");
filelist{2} = sprintf("%s/%s",Folder,"OdomY");
filelist{3} = sprintf("%s/%s",Folder,"OdomT");
filelist{4} = sprintf("%s/%s",Folder,"YawT");
filelist{5} = sprintf("%s/%s",Folder,"OdomXD");
filelist{6} = sprintf("%s/%s",Folder,"OdomYD");
filelist{7} = sprintf("%s/%s",Folder,"OdomTD");
filelist{8} = sprintf("%s/%s",Folder,"YawTD");
filelist{9} = sprintf("%s/%s",Folder,"LeftWheelSpeed");
filelist{10} = sprintf("%s/%s",Folder,"RightWheelSpeed");
filelist{11} = sprintf("%s/%s",Folder,"LeftWheelTick");
filelist{12} = sprintf("%s/%s",Folder,"RightWheelTick");
data{1}=0;
for x = 1:12
	file = fopen(filelist{x},"r");
	data{x} = fscanf(file,"%f");
	fclose(file);
endfor
	
endfunction
