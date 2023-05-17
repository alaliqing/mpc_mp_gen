%  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  */
function[] = genmprim_cart(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%

%defines

UNICYCLE_MPRIM_16DEGS = 1;


if UNICYCLE_MPRIM_16DEGS == 1
    resolution = 0.03;
    numberofangles = 16; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 11;
    numberofjointangles = 16;

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 1;
    backwardcostmult = 1;
    forwardandturncostmult225 = 1;
    forwardandturncostmult45 = 3;
    backwardandturncostmult225 = 1;
    backwardandturncostmult45 = 3;
    jackknifecostmult = 100000;
    %sidestepcostmult = 10;
    %turninplacecostmult = 5;
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 6); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts0_c(1,:) = [10 0 0 1 forwardcostmult 0];
    basemprimendpts0_c(2,:) = [20 0 0 1 forwardcostmult 0];
    basemprimendpts0_c(3,:) = [-15 0 0 1 backwardcostmult 0];    
    %1/32 theta change(+-45)
    basemprimendpts0_c(4,:) = [28 12 2 1 forwardandturncostmult45 2];
    basemprimendpts0_c(5,:) = [28 -12 -2 1 forwardandturncostmult45 -2];
    %2/32 theta change (+-22.5)
    basemprimendpts0_c(6,:) = [28 5 1 1 forwardandturncostmult225 1];
    basemprimendpts0_c(7,:) = [28 -5 -1 1 forwardandturncostmult225 -1];
    %1/32 theta change going backward(+-45)
    basemprimendpts0_c(8,:) = [-28 -12 2 1 backwardandturncostmult45 -2];
    basemprimendpts0_c(9,:) = [-28 12 -2 1 backwardandturncostmult45 2];
    %2/32 theta change going backward(+-225)
    basemprimendpts0_c(10,:) = [-28 -5 1 1 backwardandturncostmult225 -1];
    basemprimendpts0_c(11,:) = [-28 5 -1 1 backwardandturncostmult225 1];
    
    
    %45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 6); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change 
    basemprimendpts45_c(1,:) = [10 10 0 1 forwardcostmult 0];
    basemprimendpts45_c(2,:) = [20 20 0 1 forwardcostmult 0];
    basemprimendpts45_c(3,:) = [-15 -15 0 1 backwardcostmult 0];    
    %1/32 theta change(+-45)
    basemprimendpts45_c(4,:) = [12 28 2 1 forwardandturncostmult45 2];
    basemprimendpts45_c(5,:) = [28 12 -2 1 forwardandturncostmult45 -2]; 
    %2/32 theta change(+-22.5)
    basemprimendpts45_c(6,:) = [16 24 1 1 forwardandturncostmult225 1];
    basemprimendpts45_c(7,:) = [24 16 -1 1 forwardandturncostmult225 -1];
    %1/32 theta change going back)(+-45)
    basemprimendpts45_c(8,:) = [-12 -28 2 1 backwardandturncostmult45 -2];
    basemprimendpts45_c(9,:) = [-28 -12 -2 1 backwardandturncostmult45 2]; 
    %2/32 theta change going back(+-225)
    basemprimendpts45_c(10,:) = [-16 -24 1 1 backwardandturncostmult225 -1];
    basemprimendpts45_c(11,:) = [-24 -16 -1 1 backwardandturncostmult225 1]; 
    
    %22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 6); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts22p5_c(1,:) = [16 8 0 1 forwardcostmult 0];
    basemprimendpts22p5_c(2,:) = [24 12 0 1 forwardcostmult 0];    
    basemprimendpts22p5_c(3,:) = [-20 -10 0 1 backwardcostmult 0];    
    %1/32 theta change(+-45)
    basemprimendpts22p5_c(4,:) = [22 22 2 1 forwardandturncostmult45 2];
    basemprimendpts22p5_c(5,:) = [31 1 -2 1 forwardandturncostmult45 -2];    
    %2/32 theta change(+-22.5)
    basemprimendpts22p5_c(6,:) = [24 16 1 1 forwardandturncostmult225 1];
    basemprimendpts22p5_c(7,:) = [28 5 -1 1 forwardandturncostmult225 -1]; 
    %1/32 theta change going back(+-45)
    basemprimendpts22p5_c(8,:) = [-22 -22 2 1 backwardandturncostmult45 -2];
    basemprimendpts22p5_c(9,:) = [-31 -1 -2 1 backwardandturncostmult45 2];
    %2/32 theta change going back(+-225)
    basemprimendpts22p5_c(10,:) = [-24 -16 1 1 backwardandturncostmult225 -1];
    basemprimendpts22p5_c(11,:) = [-28 -5 -1 1 backwardandturncostmult225 1];
    
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'numberofjointangles: %d\n', 5);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles*5);

%iterate over angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        
        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c(1:3) = basemprimendpts0_c(primind,1:3);
            basemprimendpts_c(5:6) = basemprimendpts0_c(primind,5:6);
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)  
            basemprimendpts_c(1:3) = basemprimendpts45_c(primind,1:3);
            basemprimendpts_c(5:6) = basemprimendpts45_c(primind,5:6);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c(1:3) = basemprimendpts22p5_c(primind,1:3);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            basemprimendpts_c(5) = basemprimendpts22p5_c(primind,5);
            basemprimendpts_c(6) = -basemprimendpts22p5_c(primind,6);
            %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
            %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n'); 
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c(1:3) = basemprimendpts22p5_c(primind,1:3);
            basemprimendpts_c(5:6) = basemprimendpts22p5_c(primind,5:6);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        %iterate over joint angles
        for joint = 1:5
            fprintf(fout, 'primID: %d\n', primind-1);
            fprintf(fout, 'startangle_c: %d\n', angleind-1);
            fprintf(fout, 'startjointangle_c: %d\n', joint-1);
            
            %current jointangles
            currentjointangle = (joint-3)*2*pi/numberofjointangles;
            currentjointangle_36000int = round((joint-3)*36000/numberofjointangles);
            
            %compute which template to use
            if (rem(currentjointangle_36000int, 9000) == 0)
                basemprimendpts_c(4) = basemprimendpts0_c(primind,4);    
                jointangle = currentjointangle;
                fprintf(1, '0p\n');
                fprintf(1, 'shi0p\n');
            elseif (rem(currentjointangle_36000int, 4500) == 0)  
                basemprimendpts_c(4) = basemprimendpts45_c(primind,4);
                jointangle = currentjointangle - 45*pi/180;
                fprintf(1, '45p\n');
                fprintf(1, 'shi45p\n');
            elseif (rem(currentjointangle_36000int-6750, 9000) == 0)
                basemprimendpts_c(4) = basemprimendpts22p5_c(primind, 4); %reverse the angle as well
                %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
                %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
                jointangle = currentjointangle - 67.5*pi/180;
                fprintf(1, '67p5\n'); 
            elseif (rem(currentjointangle_36000int-2250, 9000) == 0)
                basemprimendpts_c(4) = basemprimendpts22p5_c(primind,4);
                jointangle = currentjointangle - 22.5*pi/180;
                fprintf(1, '22p5\n');
                fprintf(1, 'shi22p5\n');
            else
                fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentjointangle_36000int);
                return;
            end;
            
            %TODO:for the degree 67.5
            %if(rem(currentjointangle_36000int-6750, 9000) == 0)
            %    basemprimendpts0_c(primind,4) = -basemprimendpts0_c(primind,4);
            %end
            %determine the change direction of the joint angle 
            %move forward
            if basemprimendpts_c(3) == 0
                if((jointangle == 0 & basemprimendpts_c(1) > 0) | (jointangle < 0 & basemprimendpts_c(1) < 0))
                    basemprimendpts_c(4) = -basemprimendpts0_c(primind,4);
                else
                    basemprimendpts_c(4) = basemprimendpts0_c(primind,4);
                end;
            end;
            %move forward/backward and turn
            if(basemprimendpts_c(6) > 0)
                basemprimendpts_c(4) = -basemprimendpts0_c(primind,4);
            end;
            %now figure out what action will be        
            baseendpose_c = basemprimendpts_c(1:4);
            additionalactioncostmult = basemprimendpts_c(5);        
            endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
            endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
            endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
            endphi_c = rem(joint - 1 + baseendpose_c(4), numberofjointangles);
            endpose_c = [endx_c endy_c endtheta_c endphi_c];

            fprintf(1, 'rotation angle=%f\n', angle*180/pi);
            fprintf(1, 'joint angle=%f\n', jointangle*180/pi);
            
            %TODO:limit the movement of the jackknife state when move
            %backward,set large cost
            %move backward and no turn
%             if (joint==1&endpose_c(4) ==  | joint==5) & baseendpose_c(3) == 0
%                 additionalactioncostmult = jackknifecostmult;
%             end
%             %move backward with turn
%             if joint==1 & basemprimendpts_c(6) < 0 & basemprimendpts_c(1) < 0
%                 additionalactioncostmult = jackknifecostmult;
%             end
%             if joint==5 & basemprimendpts_c(6) > 0 & basemprimendpts_c(1) < 0
%                 additionalactioncostmult = jackknifecostmult;
%             end
% 
%             if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
%                 %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
%             end;

            %generate intermediate poses (remember they are w.r.t 0,0 (and not
            %centers of the cells)
            numofsamples = 10;
            intermcells_m = zeros(numofsamples,4);
            if UNICYCLE_MPRIM_16DEGS == 1
                startpt = [0 0 currentangle currentjointangle];
                endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                    rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles ...
                    rem(joint - 3 + baseendpose_c(4), numberofjointangles)*2*pi/numberofjointangles];
                %add jointangle constraints (revolute)
                if(endpt(4) < rem(-2, numberofjointangles)*2*pi/numberofjointangles)
                    endpt(4) = startpt(4);
                    endpose_c(4) = joint -1;
                end;
                if(endpt(4) > rem(2, numberofjointangles)*2*pi/numberofjointangles)
                    endpt(4) = startpt(4);
                    endpose_c(4) = joint -1;
                end;
                if (basemprimendpts_c(6) == 1)
                    if (endpt(4) < rem(-1, numberofjointangles)*2*pi/numberofjointangles)
                        endpt(4) = startpt(4);
                        endpose_c(4) = joint -1;
                    end
                    if (startpt(4) == rem(-2,numberofjointangles)*2*pi/numberofjointangles)
                        endpt(4) = rem(-1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = joint;
                    end
                    %%add changed resolution 08.08 update
                    if joint==4 %% 25 to stable point
                        endpt(4) = rem(-1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = joint -3;
                    end
                    if joint==5
                        endpt(4) = rem(-1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = joint -4;
                    end
                end
                if (basemprimendpts_c(6) == -1) 
                    if (endpt(4) > rem(1, numberofjointangles)*2*pi/numberofjointangles)
                        endpt(4) = startpt(4);
                        endpose_c(4) = joint -1;
                    end
                    if (startpt(4) == rem(2,numberofjointangles)*2*pi/numberofjointangles)
                        endpt(4) = rem(1,numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = joint - 2;
                    end
                    %%add changed resolution 08.08 update
                    if joint==1
                        endpt(4) = rem(1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 3;
                    end
                    if joint==2
                        endpt(4) = rem(1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 3;
                    end
                end
                
                %%change the resolution of 225 and 45 08.08update
                if (basemprimendpts_c(6) == 2)
                    if joint ==3
                        endpt(4) = rem(-2, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 0;
                    end
                    if joint ==4
                        endpt(4) = rem(-1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 1;
                    end
                    if joint ==5
                        endpt(4) = rem(-1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 1;
                    end
                end
                if (basemprimendpts_c(6) == -2)
                    if joint ==1
                        endpt(4) = rem(1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 3;
                    end
                    if joint ==2
                        endpt(4) = rem(1, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 3;
                    end
                    if joint ==3
                        endpt(4) = rem(2, numberofjointangles)*2*pi/numberofjointangles;
                        endpose_c(4) = 4;
                    end
                end
                
                %TODO:limit the movement of the jackknife state when move
                %backward,set large cost
                %move backward and no turn
                if (joint==1 | joint==5) & baseendpose_c(3) == 0 & endpt(4) ==startpt(4)
                    additionalactioncostmult = jackknifecostmult;
                end
                %move backward with turn
                if joint<3 & basemprimendpts_c(6) < 0 & basemprimendpts_c(1) < 0 
                    additionalactioncostmult = jackknifecostmult;
                end
                if joint>3 & basemprimendpts_c(6) > 0 & basemprimendpts_c(1) < 0 
                    additionalactioncostmult = jackknifecostmult;
                end

                if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
                    %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
                end;
                
                
                intermcells_m = zeros(numofsamples,4);
                if ((endx_c == 0 & endy_c == 0) | baseendpose_c(3) == 0) %turn in place or move forward            
                    for iind = 1:numofsamples
                        intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                                startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                                rem(startpt(3) + (endpt(3) - startpt(3))*(iind-1)/(numofsamples-1), 2*pi) ...
                                                rem(startpt(4) + (endpt(4) - startpt(4))*(iind-1)/(numofsamples-1), 2*pi)];
                        rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                        intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
                        %if tractor and trailer move forward with phi = 0
                        %and phi=pi
                        if (currentjointangle == 0 & baseendpose_c(3) == 0)
                            intermcells_m(iind,4) = 0;
                            endpose_c(4) = rem(joint - 1,numberofjointangles);
                        elseif(currentjointangle == pi & baseendpose_c(3) == 0)
                            intermcells_m(iind,4) = pi;
                            endpose_c(4) = rem(joint -1,numberofjointangles);
                        end;
                        %when the tractor turn in place, trailer turn in
                        %different direction
                        if(baseendpose_c(3) == 1)
                            baseendpose_c(4) = -basemprimendpts0_c(primind,4);
                            endpt(4) = rem(joint - 3 + baseendpose_c(4), numberofjointangles)*2*pi/numberofjointangles;
                            %consider the jointangle constraints at 0
                            if(endpt(4) == rem(-1, numberofjointangles)*2*pi/numberofjointangles)
                                endpt(4) = startpt(4);
                            end;
                            intermcells_m(iind,4) = rem(startpt(4) - (endpt(4) - startpt(4))*(iind-1)/(numofsamples-1), 2*pi);
                        end;
                    end;            
                else %unicycle-based move forward or backward
                    R = [cos(startpt(3)) sin(endpt(3)) - sin(startpt(3));
                        sin(startpt(3)) -(cos(endpt(3)) - cos(startpt(3)))];
                    S = pinv(R)*[endpt(1) - startpt(1); endpt(2) - startpt(2)];
                    l = S(1); 
                    tvoverrv = S(2);
                    rv = (baseendpose_c(3)*2*pi/numberofangles + l/tvoverrv);
                    tv = tvoverrv*rv;

                    if ((l < 0 & tv > 0) | (l > 0 & tv < 0))
                        fprintf(1, 'WARNING: l = %d < 0 -> bad action start/end points\n', l);
                        l = 0;
                    end;
                    %differential drive model
                    
                    %compute rv
                    %rv = baseendpose_c(3)*2*pi/numberofangles;
                    %compute tv
                    %tvx = (endpt(1) - startpt(1))*rv/(sin(endpt(3)) - sin(startpt(3)));
                    %tvy = -(endpt(2) - startpt(2))*rv/(cos(endpt(3)) - cos(startpt(3)));
                    %tv = (tvx + tvy)/2.0; 
                  
                    %generate samples
                    for iind = 1:numofsamples                                        
                        dt = (iind-1)/(numofsamples-1);

                        dtheta = rv*dt + startpt(3);
                        intermcells_m(iind,:) = [startpt(1) + tv/rv*(sin(dtheta) - sin(startpt(3))) ...
                                                startpt(2) - tv/rv*(cos(dtheta) - cos(startpt(3))) ...
                                                dtheta ...
                                                 startpt(4)];

                    %    if(abs(dt*tv) < abs(l))
                    %        intermcells_m(iind,:) = [startpt(1) + dt*tv*cos(startpt(3)) ...
                    %                                 startpt(2) + dt*tv*sin(startpt(3)) ...
                    %                                 startpt(3)...
                    %                                 startpt(4)];
                    %    else
                    %        dtheta = rv*(dt - l/tv) + startpt(3);
                    %        intermcells_m(iind,:) = [startpt(1) + l*cos(startpt(3)) + tvoverrv*(sin(dtheta) - sin(startpt(3))) ...
                    %                                 startpt(2) + l*sin(startpt(3)) - tvoverrv*(cos(dtheta) - cos(startpt(3))) ...
                    %                                 dtheta...
                    %                                 startpt(4)];
                    %    end;
                    end; 
                    %correct
                    errorxy = [endpt(1) - intermcells_m(numofsamples,1) ... 
                               endpt(2) - intermcells_m(numofsamples,2) ...
                               endpt(4) - intermcells_m(numofsamples,4)];
                    fprintf( 'l=%f errx=%f erry=%f errphi=%f\n', errorxy(1), errorxy(2), errorxy(3));
                    interpfactor = [0:1/(numofsamples-1):1];
                    intermcells_m(:,1) = intermcells_m(:,1) + errorxy(1)*interpfactor';
                    intermcells_m(:,2) = intermcells_m(:,2) + errorxy(2)*interpfactor';
                    intermcells_m(:,4) = intermcells_m(:,4) + errorxy(3)*interpfactor';
                    
                end;                                        
            end;

            %write out
            fprintf(fout, 'endpose_c: %d %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3), endpose_c(4));
            fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
            fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
            for interind = 1:size(intermcells_m, 1)
                fprintf(fout, '%.4f %.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3), intermcells_m(interind,4));
            end;

            plot(intermcells_m(:,1), intermcells_m(:,2));
            axis([-0.3 0.3 -0.3 0.3]);
            text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
            hold on;
        end;
    end;
    grid;
    pause;
end;
        
fclose('all');