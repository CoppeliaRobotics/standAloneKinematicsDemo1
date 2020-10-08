#include "ik.h"
#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "extApi.h"
}

// For double-precision, define IK_DOUBLE in the project settings

int main(int argc, char* argv[])
{
    int portNb=0;
    int simMotorHandles[7];
    int simMotorHandles2[7];
    if (argc>=9+7)
    { // We get the port and the motor handles (for the visual feedback, not for IK!) via command-line arguments
        portNb=atoi(argv[1]);
        simMotorHandles[0]=atoi(argv[2]);
        simMotorHandles[1]=atoi(argv[3]);
        simMotorHandles[2]=atoi(argv[4]);
        simMotorHandles[3]=atoi(argv[5]);
        simMotorHandles[4]=atoi(argv[6]);
        simMotorHandles[5]=atoi(argv[7]);
        simMotorHandles[6]=atoi(argv[8]);
        simMotorHandles2[0]=atoi(argv[9]);
        simMotorHandles2[1]=atoi(argv[10]);
        simMotorHandles2[2]=atoi(argv[11]);
        simMotorHandles2[3]=atoi(argv[12]);
        simMotorHandles2[4]=atoi(argv[13]);
        simMotorHandles2[5]=atoi(argv[14]);
        simMotorHandles2[6]=atoi(argv[15]);
    }
    else
    {
        printf("Indicate following arguments: 'portNumber robot1Motor1Handle... robot1motor7Handle robot2Motor1Handle... robot2motor7Handle'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Read the kinematic file:
    FILE *file;
    file=fopen("lbr_iiwa_7_r800.ik","rb");
    unsigned char* data=NULL;
    int dataLength=0;
    if (file)
    {
        fseek(file,0,SEEK_END);
        unsigned long fl=ftell(file);
        dataLength=(int)fl;
        fseek(file,0,SEEK_SET);
        data=new unsigned char[dataLength];
        fread((char*)data,dataLength,1,file);
        fclose(file);
    }
    else
    {
        printf("The kinematic content file 'lbr.ik' could not be read!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // Initialize the Coppelia Kinematics Routines robot model1:
    int ikEnvironment1Handle;
    ikCreateEnvironment(&ikEnvironment1Handle);
    ikLoad(data,dataLength);

    // Initialize the Coppelia Kinematics Routines robot model2:
    int ikEnvironment2Handle;
    ikCreateEnvironment(&ikEnvironment2Handle);
    ikLoad(data,dataLength);

    delete[] data;
    ikSwitchEnvironment(ikEnvironment1Handle); // use robot model1

    // Connect to CoppeliaSim at the above specified port, via the remote API. CoppeliaSim is just used for visual feed-back, not IK calculation!
    int clientID=simxStart("127.0.0.1",portNb,true,true,2000,5);
    if (clientID!=-1)
    {
        float simulationStep;
        simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&simulationStep,simx_opmode_streaming);

        simxSynchronous(clientID,1); // We enable the synchronous mode, so that we can trigger each simulation step from here

        int model1MotorHandles[7];
        ikGetObjectHandle("LBR_iiwa_7_R800_joint1",model1MotorHandles+0);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint2",model1MotorHandles+1);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint3",model1MotorHandles+2);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint4",model1MotorHandles+3);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint5",model1MotorHandles+4);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint6",model1MotorHandles+5);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint7",model1MotorHandles+6);
        int model1TargetHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800_target",&model1TargetHandle);
        int model1BaseHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800",&model1BaseHandle);

        simReal v=0.0;

        // Get the initial target dummy transformation, of the embedded model1:
        C7Vector dummy1Transf;
        ikGetObjectTransformation(model1TargetHandle,model1BaseHandle,&dummy1Transf);

        int model2MotorHandles[7];
        ikGetObjectHandle("LBR_iiwa_7_R800_joint1",model2MotorHandles+0);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint2",model2MotorHandles+1);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint3",model2MotorHandles+2);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint4",model2MotorHandles+3);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint5",model2MotorHandles+4);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint6",model2MotorHandles+5);
        ikGetObjectHandle("LBR_iiwa_7_R800_joint7",model2MotorHandles+6);
        int model2TargetHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800_target",&model2TargetHandle);
        int model2BaseHandle;
        ikGetObjectHandle("LBR_iiwa_7_R800",&model2BaseHandle);

        simReal v2=0.0;

        // Get the initial target dummy transformation, of the embedded model2:
        C7Vector dummy2Transf;
        ikGetObjectTransformation(model2TargetHandle,model2BaseHandle,&dummy2Transf);


        while (simxGetConnectionId(clientID)!=-1)
        {
            // Following 3 commands will slow down the simulation, but garantee that if the simulation time step was changed,
            // that there won't be any jumps. Following 3 commands are not needed if you don't modify the simulation time step
            // (i.e. dt) during simulation.
            simxUChar simWaitingForTrigger=0;
            while ( (simWaitingForTrigger==0)&&(simxGetConnectionId(clientID)!=-1) )
                simxGetBooleanParameter(clientID,sim_boolparam_waiting_for_trigger,&simWaitingForTrigger,simx_opmode_blocking);

            simxGetFloatingParameter(clientID,sim_floatparam_simulation_time_step,&simulationStep,simx_opmode_buffer);
            v+=simReal(0.6)*simReal(simulationStep);
            v2+=simReal(0.84)*simReal(simulationStep);


            ikSwitchEnvironment(ikEnvironment1Handle); // use robot model1
            // Set the desired tip transformation by setting the target dummy transformation:
            dummy1Transf.X=C3Vector(simReal(-0.3-cos(v)*0.1),simReal(sin(v)*0.1),simReal(0.629+sin(v*9)*0.01));
            ikSetObjectTransformation(model1TargetHandle,model1BaseHandle,&dummy1Transf);

            // calculate IK:
            ikHandleIkGroup(ik_handle_all);

            // Read the corresponding motor angles and send them to CoppeliaSim:
            simxPauseCommunication(clientID,1); // Temporarily pause the remote API communication, in order to send all following commands at once
            simReal pos;
            ikGetJointPosition(model1MotorHandles[0],&pos);
            simxSetJointPosition(clientID,simMotorHandles[0],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[1],&pos);
            simxSetJointPosition(clientID,simMotorHandles[1],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[2],&pos);
            simxSetJointPosition(clientID,simMotorHandles[2],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[3],&pos);
            simxSetJointPosition(clientID,simMotorHandles[3],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[4],&pos);
            simxSetJointPosition(clientID,simMotorHandles[4],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[5],&pos);
            simxSetJointPosition(clientID,simMotorHandles[5],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model1MotorHandles[6],&pos);
            simxSetJointPosition(clientID,simMotorHandles[6],(float)pos,simx_opmode_oneshot);

            ikSwitchEnvironment(ikEnvironment2Handle); // use robot model2
            // Set the desired tip transformation by setting the target dummy transformation:
            dummy2Transf.X(0)=simReal(-0.3-cos(v2)*0.1);
            dummy2Transf.X(1)=simReal(sin(v2)*0.1);
            ikSetObjectTransformation(model2TargetHandle,model2BaseHandle,&dummy2Transf);

            // calculate IK:
            ikHandleIkGroup(ik_handle_all);

            // Read the corresponding motor angles and send them to CoppeliaSim:
            ikGetJointPosition(model2MotorHandles[0],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[0],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[1],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[1],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[2],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[2],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[3],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[3],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[4],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[4],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[5],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[5],(float)pos,simx_opmode_oneshot);
            ikGetJointPosition(model2MotorHandles[6],&pos);
            simxSetJointPosition(clientID,simMotorHandles2[6],(float)pos,simx_opmode_oneshot);

            simxPauseCommunication(clientID,0); // Unpause the remote API communication

            // Now step the simulation on CoppeliaSim side:
            int r=simx_return_remote_error_flag; // means for next remote API function call: step not triggered
            while (r==simx_return_remote_error_flag)
                r=simxSynchronousTrigger(clientID); // Trigger next simulation step
            if (r!=simx_return_ok)
                break;

            printf(".");
        }
        ikEraseEnvironment();
        ikEraseEnvironment();
        simxFinish(clientID); // End the remote API
    }
    return(0);
}
