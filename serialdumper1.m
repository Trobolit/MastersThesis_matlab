
comport = 'COM3';
delete(instrfind('Port',comport));

%open serial object
s = serial(comport);
s.BaudRate = 115200;
try
    fopen(s);

    %Initialize the variable to store the data which is to be received
    data = [];
    
    for i=1:1000


        %wait for half a sec before reading the data sent by the board.
        %pause(0.01);
    
        %Read the accelerometer data sent by the pixhawk board. The
        %accelerometer data are 3 single precision (but the quat 4 single precision)
        %values(12 bytes). The header
        %associated with this data is [5 5]. Hence expectedDataLength = 14
        %byte = 6; %QUE
        %single
        expectedDataLength = 17; %4*6;
        data = fread(s,expectedDataLength);
        
        if isempty(data)
            disp('Unable to read any data.');
        else

        %Logic to find the packet after stripping the header
        data = data';

        %display the accel data on screen
            %disp(typecast(uint8(data),'single'))
            %disp(uint8(data(1)));
            %disp(data(2:5));
            %disp(typecast(data(2),'single'));
            %disp(uint8(data(6)));
            %disp(data);
            %disp(typecast(uint8(data),'single'));
        %disp(data(1));
        disp(typecast(uint8(data(2:end)),'single'));
        %disp(data(end));
        end
    end
       
    %close the connection after reading
    fclose(s);
    delete(s);
    clear s;
    
catch ME
    disp(ME.message)
end

