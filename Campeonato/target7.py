targetName = 'target7'
[erro, target] = sim.simxGetObjectHandle(clientID, targetName, sim.simx_opmode_oneshot_wait)
[erro, [xt,yt,zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_streaming)

ea = 0; acuma = 0
ef = 0; acumf = 0

run = True
while run == True:
 
    [erro, [xr, yr, zr]] = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer)
    [erro, [xt, yt, zt]] = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_buffer)
    [erro, [alpha, beta, gamma]] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer)

    distAtual = math.sqrt((xt - xr) ** 2 + (yt - yr) ** 2)
    angSr = math.atan2(yt - yr, xt - xr)
    erroAng = fun.norm(gamma - angSr)

    if distAtual > disTolMax and estado == 'stopped':
        estado = 'moving'
    elif distAtual < disTolMin and estado == 'moving':
        estado = 'stopped'

    if estado == 'stopped':
        sim.simxPauseCommunication(clientID, True)
        sim.simxSetJointTargetVelocity(clientID, robotRightMotor, 0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, robotLeftMotor, 0, sim.simx_opmode_oneshot)
        sim.simxPauseCommunication(clientID, False)
        run = False
    elif estado == 'moving':
        #PID Angles
        eOlda = ea
        ea = fun.norm(angSr - gamma)
        acuma = acuma + (ea + eOlda) / 2*ts
        velA = kpa*ea + kia*acuma + kda*(ea - eOlda) / ts
        if velA > velASat:
            acuma = acuma - (velA - velASat)/kia
            velA = velASat

        #PID Distance
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

execfile('target1.py')
