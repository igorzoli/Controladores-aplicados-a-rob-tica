#coding=utf-8
import sim
import math
import time
import fun

#Começo da API
print('Started')
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) #conexão
robotName = 'lumibot'
targetName = 'Target1'
if clientID != -1:
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
    print ('Conectou com o coopelia')

    #Coleta de handles dos objetos
    [erro, robot] = sim.simxGetObjectHandle(clientID, robotName + '_body', sim.simx_opmode_oneshot_wait)
    [erro, target] = sim.simxGetObjectHandle(clientID, targetName, sim.simx_opmode_oneshot_wait)
    [erro, robotLeftMotor] = sim.simxGetObjectHandle(clientID, robotName +'_leftMotor', sim.simx_opmode_oneshot_wait)
    [erro, robotRightMotor] = sim.simxGetObjectHandle(clientID, robotName +'_rightMotor', sim.simx_opmode_oneshot_wait)

    #Stream de dados
    [erro, [xr,yr,zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
    [erro, [xt,yt,zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)
    [erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming)
    time.sleep(2)

    #Sets
    estado = 'stopped'
    angTolMin = math.radians(5)
    angTolMax = math.radians(15)
    disTolMin = 0.025
    disTolMax = 0.025
    ea = 0; acuma = 0
    ef = 0; acumf = 0
    kpa = 9; kia = 0.10; kda = 0.025; ts = 0.005;  
    kpf = 13; kif = 0.010; kdf = 0;

    run = True
    while run == True:

        # Coletar dados 
        [erro, [xr, yr, zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
        [erro, [xt, yt, zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
        [erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

        #Calculos
        distAtual = math.sqrt((xt - xr) ** 2 + (yt - yr) ** 2)
        angSr = math.atan2(yt - yr, xt - xr)
        erroAng = fun.norm(gamma - angSr)

        #Maq de estados
        if distAtual > disTolMax and estado == 'stopped':
            estado = 'moving'
        elif distAtual < disTolMin and estado == 'moving':
            estado = 'stopped'

        #Acoes
        if estado == 'stopped':
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, False)
            run = False
        elif estado == 'moving':
            #PID Angles
            velASat = 20
            eOlda = ea
            ea = fun.norm(angSr - gamma)
            acuma = acuma + (ea + eOlda) / 2*ts
            velA = kpa*ea + kia*acuma + kda*(ea - eOlda) / ts
            if velA > velASat:
                acuma = acuma - (velA - velASat)/kia
                velA = velASat

            #PID Distance
            velFSat = 20
            eOldf = ef
            ef = distAtual
            acumf = acumf + (ef + eOldf) / 2*ts
            velF = kpf*ef + kif*acumf + kdf*(ef - eOldf) / ts
            if velF > velFSat:
                acumf = acumf - (velF - velFSat)/kif
                velF = velFSat
            
            velR = velF + velA
            velL = velF - velA
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velR, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, velL, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, False)
            
    # Finish simulation
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Programa pausado', sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
else:
    print('Problema de conexao com o coopelia')
