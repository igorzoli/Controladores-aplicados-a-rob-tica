#coding=utf-8
try:
    import sim
    import math
    import time

#Começo da API
    print('Started')
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) #conexão
    robotName = 'lumibot'
    targetName = 'Target1'
    if clientID != -1:
        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)
        print ('Conectou finalmente nesse krl!')

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

        run = True
        while run == True:

            # Coletar dados 
            [erro, [xr, yr, zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
            [erro, [xt, yt, zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
            [erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

            #Calculos
            distAtual = math.sqrt((xt - xr) ** 2 + (yt - yr) ** 2)
            distSet = 0.025
            angSr = math.atan2(yt - yr, xt - xr)
            erroAng = gamma - angSr
            
            #Normalize
            if erroAng > math.pi:
                while erroAng > math.pi:
                    erroAng = erroAng - (math.pi * 2)
            elif erroAng < -math.pi:
                while erroAng < -math.pi:
                    erroAng = erroAng + (math.pi * 2)

            #Sets
            angTolMin = math.radians(10)
            angTolMax = math.radians(15)
            velF = 18

            #Maq de estados
            if distAtual > distSet and estado == 'stopped':
                estado = 'align'
            elif math.fabs(erroAng) <= angTolMin and estado == 'align':
                estado = 'forward'
            elif math.fabs(erroAng) > angTolMax and estado == 'forward':
                estado = 'align'
            elif distAtual < distSet and estado == 'forward':
                estado = 'stopped'
        
            if estado == 'stopped':
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0, sim.simx_opmode_oneshot)
                sim.simxPauseCommunication(clientID, False)
                run = False
            elif estado == 'align':
                velF = 4
                if (erroAng > 0):
                    sim.simxPauseCommunication(clientID, True)
                    sim.simxSetJointTargetVelocity(clientID, robotRightMotor, -velF, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, velF, sim.simx_opmode_oneshot)
                    sim.simxPauseCommunication(clientID, False)
                elif (erroAng < 0):
                    sim.simxPauseCommunication(clientID, True)
                    sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velF, sim.simx_opmode_oneshot)
                    sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, -velF, sim.simx_opmode_oneshot)
                    sim.simxPauseCommunication(clientID, False)
            elif estado == 'forward':
                sim.simxPauseCommunication(clientID, True)
                sim.simxSetJointTargetVelocity(clientID, robotRightMotor, velF, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, velF, sim.simx_opmode_oneshot)
                sim.simxPauseCommunication(clientID, False)
        # Finish simulation
        sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxAddStatusbarMessage(clientID, 'Programa pausado', sim.simx_opmode_blocking)
        sim.simxFinish(clientID)
    else:
        print('Problema de conexão com o coopelia')
except:
    print('Acabou essa merda, Lembrete: caso dê errado dessa vez desistir!')