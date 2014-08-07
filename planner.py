# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history


class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec):
		buchi = mission_to_buchi(hard_spec, soft_spec)
		self.product = ProdAut(ts, buchi)
		self.time = 0
		self.pose = None
		self.trace = [] # log the regions been visited
		self.traj = [] # log the full trajectory
		self.opt_log = [] 
		self.com_log = []

	def optimal(self, beta=10, style='static'):
		self.beta = beta
		if style == 'static':
			# full graph construction
			self.product.graph['ts'].build_full()
			self.product.build_full()
			self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
		elif style == 'on-the-fly':
			# on-the-fly construction
			self.product.build_initial()
			self.product.build_accept()
			self.run, plantime = dijkstra_plan_optimal(self.product, self.beta)
		### show the results
		print 'the plan prefix:\n'
		print [n for n in self.run.pre_plan]
		print 'the plan suffix:\n'
		print [n for n in self.run.suf_plan]
		self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
		self.last_time = self.Time
		self.acc_change = 0
		self.index = 0
		self.segment = 'line'
		self.next_move = self.run.pre_plan[self.index]
		return plantime

	def find_next_move(self):
		if self.segment == 'line' and self.index < len(self.run.pre_plan)-1:
			self.index += 1
			self.trace.append(self.run.line[self.index])
			self.next_move = self.run.pre_plan[self.index]
		elif self.segment == 'line' and self.index == len(self.run.pre_plan)-1:
			self.trace.append(self.run.line[self.index+1])
			self.index = 0
			self.segment = 'loop'
			self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-1:
			self.index += 1
			self.trace.append(self.run.loop[self.index])
			self.next_move = self.run.suf_plan[self.index]
		elif self.segment == 'loop' and self.index == len(self.run.suf_plan)-1:
			self.trace.append(self.run.loop[self.index+1])
			self.index = 0
			self.next_move = self.run.suf_plan[self.index]
		return self.next_move


	def update(self,object_name):
		MotionFts = self.product.graph['ts'].graph['region']
		cur_region = MotionFts.closest_node(self.pose)
		sense_info = dict()
		sense_info['label'] = set([(cur_region,set([object_name,]),set()),]) 
		changes = MotionFts.update_after_region_change(sense_info,None)
		if changes:
			return True

	def replan(self):
		new_run = improve_plan_given_history(self.product, self.trace, self.pose)
		if (new_run) and (new_run.pre_plan !=self.run.pre_plan[self.index:]):
			self.run = new_run
			self.index = 0
			self.segment = 'line'
			self.next_move = self.run.pre_plan[self.index]
			print 'Plan adapted!'
		else:
			print 'Plan unchanged!'











