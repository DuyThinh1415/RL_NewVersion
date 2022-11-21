import math
import numpy as np


class GameSettingParam:
    CAPTION = "Reinforcement Learning"
    WIDTH = 400
    HEIGHT = 1500
    # HEIGHT = 1800
    FPS = 10
    
    DRAW = False
    LOCK_QTable = False
    dt = float(1/FPS)


class PlayerParam:
    RADIUS_OBJECT = 10

    ACCELERATION_FORWARD = 50
    ACCELERATION_ROTATE = math.radians(18)

    WIDTH = 16
    HEIGHT = 30

    INITIAL_X = GameSettingParam.WIDTH//2
    INITIAL_Y = GameSettingParam.HEIGHT - 20

    MAX_VELOCITY = 50
    MAX_ROTATION_VELOCITY = 2

    FOV = math.pi
    HALF_FOV = FOV/2
    # CASTED_RAYS = 30
    # STEP_ANGLE = FOV / CASTED_RAYS
    RADIUS_LIDAR = 180

    INC_ROTATION_VELO = "INC_ROTATION_VELO"
    DESC_ROTATION_VELO = "DESC_ROTATION_VELO"
    DO_NOTHING = "DO_NOTHING"
    INC_FORWARD_VELO = "INC_FORWARD_VELO"
    DESC_FORWARD_VELO = "DESC_FORWARD_VELO"
    
    INFINITY = 9999


class ObstacleParam:
    NUMBER_OF_OBSTACLES = 15
    OBSTACLE_ACCELERATION_FORWARD = 0
    OBSTACLE_ACCELERATION_ROTATE = 0
    INITIAL_OBSTACLE_X = GameSettingParam.WIDTH//2
    INITIAL_OBSTACLE_Y = 0
    
    PROBABILITIES_ACTION = [0.4,
                            0.1,
                            0.1,
                            0.2,
                            0.2]
    # PROBABILITIES_ACTION = [0.001,0.001,0.001,0.001,0.001]

class RLParam:
    EPSILON = 0.5

    MAX_ALPHA = 0.2
    MIN_ALPHA = 0.5

    GAMMA = 0.8

    AREA_RAY_CASTING_NUMBERS = 8

    N_EPISODES = 10000
    MAX_EPISODE_STEPS = 801

    GO_FUCKING_DEAD = -1000000

    # LAND_MARK_LIST = [400]*30

    # ACTIONS = [PlayerParam.INC_ROTATION_VELO,
    #            PlayerParam.DESC_ROTATION_VELO,
    #            PlayerParam.STOP,
    #            PlayerParam.INC_FORWARD_VELO,
    #            PlayerParam.DESC_FORWARD_VELO]

    # 11 22 33 44 55 66 77 88

    ACTIONS = [PlayerParam.INC_FORWARD_VELO,
               PlayerParam.INC_ROTATION_VELO,
               PlayerParam.DESC_ROTATION_VELO,
               PlayerParam.DESC_FORWARD_VELO,
               PlayerParam.DO_NOTHING]

    MAX_TIME_MS = 2*60
    
    class LEVEL_OF_RAY_CASTING:
        INFINITY = "2" # NO TOUCH OBSTACLE 
        SAFETY_DISTANCE = "1" # LIDAR TOUCH OBSTACLE, BUT SAFE
        DANGEROUS_DISTANCE = "0" # LIDAR TOUCH OBSTACLE, BUT IN DANGEROUS MODE

    class LEVEL_OF_ANGLE:
        LLEFT = "0"
        LEFT = "1"
        MID = "2"
        RIGHT = "3"
        RRIGHT = "4"

        LIST_LEVEL_ANGLES = np.array(range(10),dtype="str")

    # class Y_VER:
    #     BACK = "0"  #BACK
    #     FORD1 = "1"#0-20
    #     FORD2 = "2"#20-
    #     LIST_LEVEL_YVER = [BACK,FORD1,FORD2]

    # class OMEGA:
    #     LL = "0"
    #     L = "1"
    #     M = "2"
    #     R = "3"
    #     RR = "4"
    #     LIST_LEVER_OMEGA = [LL,L,M,R,RR]
        
    # | x | 3x | 2x | 3x | x |
    # split the middle area into 2 parts, each part will be x
    # DISTANCE_FROM_CENTER_OF_LANE = [
    #     GameSettingParam.WIDTH * 4 / 10,    # most left or most right (distance > 4x/10) (1/2 of middle + 3x area)
    #     GameSettingParam.WIDTH * 1 / 10,    # left or right (distance > x/10) (1/2 of middle)
    #     0                                   # center (distance > 0) (inside 1/2 of middle)
    # ] # 0 is lucky number, no meaning
    
    class LEVEL_OF_LANE:
        
        LIST_LEVEL_OF_LANE = np.array(range(20),dtype="str")
        
    class SCORE:
        # lidar detect obstacle
        OBSTACLE_TOUCH = -20
        DANGEROUS_ZONE_TOUCH = -1 # need to implemented
        
        # stay in middle of lane
        STAY_AT_CENTER_OF_LANE = 5 # Nhana keeu < 50
        STAY_AT_LEFT_OR_RIGHT_OF_LANE = 0
        STAY_AT_MOSTLEFT_OR_MOSTRIGHT_OF_LANE = -100
        
        # action
        STOP_ACTION = -100
        TURN_LEFT_OR_RIGHT = -1 # need to be implemented


class CustomColor:
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    PINK = (255,0,255)
    CRYAN = (0, 255, 255)


class MODE_PLAY:
    MANUAL = "MANUAL"
    RL_TRAIN = "RL_TRAIN"
    DEPLOY = "fuck this RL"

class SOME_PARAM_FOR_CODE_DO:
    passRayCasting = [0]*45
    preMinHeight = 0
    preX = 0
    preY = 0
    stateList = []