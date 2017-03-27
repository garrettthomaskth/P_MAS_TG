from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner
from P_MAS_TG.buchi import mission_to_buchi
from P_MAS_TG.product import ProdAut
from networkx_viewer import Viewer
# export PYTHONPATH=$PYTHONPATH:/to/your/P_MAS_TG

import matplotlib.pyplot as plt
import networkx as nx
import time


##############################
# motion FTS
ap = {'r1', 'r2', 'r3', 'r4', 'r5', 'r6', 'r', 'b'}
# +-----+-----+-----+
# | r4,r| r5,b| r6,b|
# +-----+-----+-----+
# | r1,r| r2,b| r3,r|
# +-----+-----+-----+
regions = {   (0, 0, 1): set(['r1', 'r']),
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

robot_motion = MotionFts(regions, ap, 'office' )
robot_motion.set_initial((0, 0, 1))
robot_motion.add_un_edges(edges, unit_cost = 0.1)


##############################
# action FTS
############# no action model
action = dict()
############# with action
#action = { 'pick': (100, 'r', set(['pick'])),
#           'drop': (50, 'b', set(['drop']))
#}


robot_action = ActionModel(action)


##############################
# complete robot model
robot_model = MotActModel(robot_motion, robot_action)



##############################
# specify tasks
########## only hard
# hard_task = '<>(r1 && <> (r2 && <> r6)) && (<>[] r6)'
#hard_task = '(<>(pick && <> drop)) && ([]<> r3) && ([]<> r6)'
#soft_task = None


########## soft and hard
hard_task = '([](<> r3) &&  []<> r2 && []<> r4)'
soft_task = None#'([]! b)'



##############################
# set planner
robot_planner = ltl_planner(robot_model, hard_task, soft_task)
robot_planner.product.graph['ts'].build_full()
robot_planner.product.build_full()
#buchi = mission_to_buchi(hard_task, soft_task)
#networkx.draw(ProdAut(robot_model, buchi))

col = [] 

i = 0

labels = {}
print(robot_planner.product.node)
for node in robot_planner.product.node:
  col.append(robot_planner.product.node[node]['color'])

  labels[node] = robot_planner.product.node[node]['color']
  
  #if node['color'] == 'r':
   # r.append(i)
  #if node['color'] == 'g':
  #  g.append(i)
  #if node['color'] == 'y':
  #  y.append(i)
print(labels)
nx.draw_networkx(robot_planner.product,node_color=col,labels=labels)
plt.show()

nx.draw_networkx(robot_planner.product.graph['buchi'])
plt.show()
#app = Viewer(robot_planner.product)
#app.mainloop()
# synthesis
start = time.time()
robot_planner.optimal(10,'static')

print 'full construction and synthesis done within %.2fs \n' %(time.time()-start)
