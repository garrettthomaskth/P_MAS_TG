from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner

# construct your motion and action model
#---------

##############################                                                  
# motion FTS                                                                    
symbols = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r', 'b'}
# +-----+-----+-----+                                                           
# | r4,r| r5,b| r6,b|                                                           
# +-----+-----+-----+                                                           
# | r1,r| r2,b| r3,r|                                                           
# +-----+-----+-----+                                                           
node_dict = {   (0, 0, 1): set(['r1', 'r']),
              (1, 0, 1): set(['r2', 'b']),
              (2, 0, 1): set(['r3', 'r']),
              (0, 1, 1): set(['r4', 'r']),
              (1, 1, 1): set(['r5', 'b']),
              (2, 1, 1): set(['r6', 'b']),
}

edges = [((0, 0, 1), (1, 0, 1)),
         ((1, 0, 1), (2, 0, 1)),
         ((0, 1, 1), (1, 1, 1)),
         ((1, 1, 1), (2, 1, 1)),
         ((0, 0, 1), (0, 1, 1)),
         ((1, 0, 1), (1, 1, 1)),
         ((2, 0, 1), (2, 1, 1)),
]



robot_motion = MotionFts(node_dict, symbols, 'office')
robot_motion.set_initial((0, 0, 1))
robot_motion.add_un_edges(edges, unit_cost = 0.1)
#---------

action_dict = { 'pick': (100, 'r', set(['pick'])),
           'drop': (50, 'b', set(['drop']))
}


robot_action = ActionModel(action_dict)
#---------
robot_model = MotActModel(robot_motion, robot_action)

# specify your hard and soft tasks
hard_task = '(([]<> r3) && ([]<> r4))'
soft_task = None

# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)

# synthesis
robot_planner.optimal(10,'static')
