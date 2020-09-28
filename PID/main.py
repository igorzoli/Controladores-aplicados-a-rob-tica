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

    estado = 'stopped'
    ea = 0; acuma = 0
    ef = 0; acumf = 0
    kpa = 5; kia = 0.0; kda = 0.0; 
    kpf = 8; kif = 0.1; kdf = 0.1;
    ts = 0.05; 

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

        #Sets
        angTolMin = math.radians(5)
        angTolMax = math.radians(10)
        disTolMin = 0.025
        disTolMax = 0.025

        #Maq de estados
        if distAtual > disTolMax and estado == 'stopped':
            estado = 'align'
        elif math.fabs(erroAng) <= angTolMin and estado == 'align':
            estado = 'forward'
        elif math.fabs(erroAng) > angTolMax and estado == 'forward':
            estado = 'align'
        elif distAtual < disTolMin and estado == 'forward':
            estado = 'stopped'

        #Acoes
        if estado == 'stopped':
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, False)
            run = False
        elif estado == 'align':
            #PID for angle
            velASat = 10
            eOlda = ea
            ea = fun.norm(angSr - gamma)
            acuma = acuma + (ea + eOlda) / 2*ts
            velA = kpa*ea + kia*acuma + kda*(ea - eOlda) / ts
            if velA > velASat:
                acuma = acuma - (velA - velASat)/kia
                velA = velASat

            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velA, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, -velA, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, False)
            
        elif estado == 'forward':
            #PID for distances
            velFSat = 20
            eOldf = ef
            ef = distAtual
            acumf = acumf + (ef + eOldf) / 2*ts
            velF = kpf*ef + kif*acumf + kdf*(ef - eOldf) / ts
            if velF > velFSat:
                acumf = acumf - (velF - velFSat)/kif
                velF = velFSat
                
            sim.simxPauseCommunication(clientID, True)
            sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velF, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, velF, sim.simx_opmode_oneshot)
            sim.simxPauseCommunication(clientID, False)
            
    # Finish simulation
    sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxAddStatusbarMessage(clientID, 'Programa pausado', sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
else:
    print('Problema de conexao com o coopelia')
