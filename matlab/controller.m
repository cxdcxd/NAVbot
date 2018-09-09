
try
cls.kill();
catch
end

clc;
clear;

prompt = 'Wait for user start';
x = input(prompt)

dllPath1 = fullfile('c:','car','Matlab.dll');
dllPath2 = fullfile('c:','car','AsyncIO.dll');
dllPath3 = fullfile('c:','car','NetMQ.dll');
dllPath4 = fullfile('c:','car','protobuf-net.dll');

asm1 = NET.addAssembly(dllPath1);
asm2 = NET.addAssembly(dllPath2);
asm3 = NET.addAssembly(dllPath3);
asm4 = NET.addAssembly(dllPath4);

cls = Matlab.MatlabInterface('127.0.0.1', '8888', '8889');
target = Matlab.RVector3(5,0,0);

try
    
message = 'Waiting for groundstation connection'

time_out = 10;
state = 1;

while (time_out > 1 && cls.is_connected == false)
    time_out = time_out - 1;
    pause(1);
    message = 'Timeout'
end

if ( cls.is_connected )

    while (state > 0 && cls.is_connected)
          
        if ( state == 1 )
            message = 'Controller Started'
            cls.setSpeed1(128);
            cls.setSpeed2(128);
            cls.setPosition(128);
            cls.setTarget(target);
            state = 2;
            pause(1);
        elseif ( state == 2 )
            
            path = cls.getPath();
            %message = strcat('Get Path Length : ',int2str(path.Length))
            
            if ( path.Length > 3 )
                
                next = path(2);
                current = cls.main_status.location;
                
                dx = next.x - current.x;
                dy = next.y - current.y;
                delta = sqrt(dx*dx + dy*dy) * 100  % Converting to cm
                deg = atan2(dy,dx);
                deg = deg - current.theta;
                deg = deg * 57.2958 * 1.5 % Converting to degree

                %============================================
                  
                   
                    position = 128;
 
                    if ( deg > 45 ) 
                        deg = 45; 
                    end
                    if ( deg < -45 )
                        deg = -45; 
                    end
                       
                    de = int16(deg);
                    position = position + de;
                    
                   
                    if ( position < 80 ) 
                        position = 80; 
                    end
                    
                    if ( position > 170) 
                        position = 170; 
                    end
                   
                     cls.setSpeed1(140);
                     cls.setSpeed2(140);
                     cls.setPosition(position);
                     cls.setTarget(target);
                
                pause(0.5);
                
            else
                message = 'No Path'
                state = 3;
            end
            
        elseif ( state == 3 )
            message = 'Goal Reached'
            cls.setSpeed1(128);
            cls.setSpeed2(128);
            cls.setPosition(128);
            state = 0;
            pause(1);
        end 
        
        
    end
    
else
 message = 'Connection Failed'   
end


catch
    
end

cls.kill();
message = 'Finished' 

    

