MODULE EGM_test_UDP
    
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_lin1:=[-1,1]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-2,2];! in degees
    CONST egm_minmax egm_minmax_joint1:=[-0.5,0.5];
    
    CONST jointtarget p20:=[[-91.08,2.54,38.18,0.0,49.27,-1.07],[9E9,9E9,9E9,9E9,9E9,9E9]];
    
    PROC main()
        EGMReset egmID1;
        EGMGetId egmID1;     
        EGMStop egmID1, EGM_STOP_HOLD, \RampOutTime := 100;
        MoveAbsJ p20,v100,fine,tool0;
        WaitTime 1;
        testuc_UDP; 
    ENDPROC
     
    PROC testuc_UDP()
        EGMReset egmID1;
        EGMGetId egmID1;
        egmSt1 := EGMGetState(egmID1);        
        
        IF egmSt1 <= EGM_STATE_CONNECTED THEN
            ! Set up the EGM data source: UdpUc server using device "EGMsensor:"and configuration "default"
            EGMSetupUC ROB_1, egmID1, "default", "EGMSensor:" \Joint \CommTimeout:=100;
        ENDIF
        
        !Which program to run
        runEGM;
              
        IF egmSt1 = EGM_STATE_CONNECTED THEN
            TPWrite "Reset EGM instance egmID1";
            EGMReset egmID1; 
        ENDIF  
    ENDPROC
              
    PROC runEGM()
        EGMActJoint egmID1 \Tool:=tool0 \WObj:=wobj0, \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
        \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1 \LpFilter:=10 \Samplerate:=4 \MaxPosDeviation:=1000 \MaxSpeedDeviation:=1000;
        
        EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=2000000 \RampInTime:=0.05 \PosCorrGain:=1;
        egmSt1:=EGMGetState(egmID1);
    ERROR
    
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "EGM UDP Command Timeout, Restarting!";
            WaitTime 5;
            ExitCycle;
        ENDIF
    ENDPROC
	
ENDMODULE
