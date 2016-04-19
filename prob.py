import db
from tools2 import *
import math

class P:
	"""This class released diferent propability"""

	debag = False
	sigmaDist = 28
	sigmaAngle = 0.71
	def __init__(self, ordinal, candidateEdge, point, pointEdge, lastOrdinal, deep, pointPath, edgePath):
		"""
		Attributes: 
			ordinal -- GPS track point
			candidateEdge -- Edge with candidate point
			point -- Last Point
			pointEdge -- Edge with point
		"""

		self.ordinal = ordinal
		self.candidateEdge = candidateEdge
		self.point = point
		self.pointEdge = pointEdge
		self.lastOrdinal = lastOrdinal
		self.candidate = self.getCandidate()
		self.shortestPath = self.calculateShortestPath(self.candidate, self.candidateEdge, deep, pointPath, edgePath)
		self.multiProb = self.calculateMultiProb()
		# self.m = self.metric(self.point, ordinal, candidateEdge)

	def getCandidate(self):
		h,u = dist(self.ordinal, self.candidateEdge)
		self.u = u
		if u < 0 or u > 1:
			if h == distance(self.ordinal, self.candidateEdge.src):
				ret = self.candidateEdge.src
				ret.time = self.ordinal.time
			else:
				ret = self.candidateEdge.trg
				ret.time = self.ordinal.time
		else:
			try:
				x = math.sqrt((self.ordinal-self.candidateEdge.src).getModule() ** 2 - h ** 2)
			except ZeroDivisionError:
				raise IOError
			except ValueError:
				print('acc err='+str((self.ordinal-self.candidateEdge.src).getModule() ** 2 - h ** 2))
				ret = self.candidateEdge.src
				ret.time = self.ordinal.time
				return ret
			ret = self.candidateEdge*(x/self.candidateEdge.getModule()) + self.candidateEdge.src
			ret.time = self.ordinal.time
		return ret

	def calculateShortestPath(self, toPoint, toEdge, deep, pointPath, edgePath):
		"""Calculate shortest path between two point

		Attributes:
			toPoint -- last point on path
			toEdge -- last edge on path
			deep -- how deep look into pointPath
			pointPath -- sequence of point on path
			edgePath -- sequence of edge on path
		"""

		firstPoint = pointPath[-deep]
		if toEdge.id == firstPoint[1]:
			if self.debag: print(distance(firstPoint[0], toPoint)*111.11111*1000)
			return distance(firstPoint[0], toPoint)

		if edgePath[-1].id == toEdge:
			endPath = 1
		else:
			endPath = 0
		end = len(edgePath) - endPath
		i = endPath
		while (i <= end):
			i += 1
			if edgePath[-i].id == firstPoint[1]:
				firstEdge = edgePath[-i]
				i -= 1
				break

		distOnFirstEdge = distance(firstPoint[0],firstEdge.trg)

		distFromFirstEdgeToLastEdge = 0
		while i > endPath:
			distFromFirstEdgeToLastEdge += edgePath[-i].getModule()
			i -= 1

		distOnLastEdge = distance(toEdge.src, toPoint)
		if distOnLastEdge == distFromFirstEdgeToLastEdge:
			distFromFirstEdgeToLastEdge = 0
		if self.debag:
			print(distOnFirstEdge*111.11111*1000)
			print(distFromFirstEdgeToLastEdge*111.11111*1000)
			print(distOnLastEdge*111.11111*1000)
		return distOnFirstEdge + distFromFirstEdgeToLastEdge+ distOnLastEdge

	def calculateObserveProb(self, ordinal, candidateEdge):
		""" Calculate observe propability

		Attributes: 
			ordinal -- GPS track point
			candidateEdge -- Candidate edge
		"""

		sigma = self.sigmaDist

		h,u = dist(ordinal, candidateEdge)
		if u < 0 or u > 1:
			distanc = distance(self.candidateEdge.trg, ordinal)*111.11111*1000
		else:
			distanc = h*111.11111*1000
		return 1/2*(1 + math.erfc(distanc/math.sqrt(2*sigma**2)))
		
	def calculateTransmissionProb(self, point, candidate, shortestPath):
		"""Calculates transmission probability for connection

		Attributes:
			point -- last point
			candidate -- Candidate on point
		"""

		distToCandidate = distance(point, candidate)
		if distToCandidate == 0 or shortestPath == 0:
			return 1
		else:
			return distToCandidate/shortestPath


	def calculateSpeedProb(self, avgSpeed, speed):
		"""Calculate speed propability"""

		sigma = 20
		return 1/2*(1 + math.erfc((speed - avgSpeed)/math.sqrt(2*sigma**2)))

	def calculateAngleProb(self, pointEdge, candidateEdge):
		"""Calculate propability change angle"""

		sigma = self.sigmaAngle
		return 1/2*(1 + math.erfc(1 - deltaAngle(pointEdge, candidateEdge))/math.sqrt(2* sigma ** 2))

	def calculateMultiProb(self):
		"""Calculate full propability and rewrite inner attributes"""
		#print(self.shortestPath*111.11111*1000)
		# if shortestPath == 0:
		# 	speed = 0
		# else:
		# 	speed = shortestPath/(self.candidate.time - self.point.time)*3.6
		# avgSpeed = (self.candidateEdge.speed + self.pointEdge.speed)/2
		self.observeProb = self.calculateObserveProb(self.ordinal, self.candidateEdge)
		# self.transmissionProb = self.calculateTransmissionProb(self.point, self.candidate, self.shortestPath)
		# self.speedProb = self.calculateSpeedProb(avgSpeed, speed)
		self.angleProb = self.calculateAngleProb(Edge(self.lastOrdinal, self.ordinal), self.candidateEdge)
		return self.observeProb * self.angleProb #* self.transmissionProb

	@staticmethod
	def metric(pointFirst, pointSecond, edge):
		def Sd(md, nd, a):
			h,u = dist(pointSecond, edge)
			return md - a*(h*111.11111*1000) ** nd 

		def Sa(ma, na):
			a = pointSecond - pointFirst
			return ma*(deltaAngle(Edge(pointFirst, pointSecond), edge)) ** na
		
		edge.src.converted = False	
		edge.trg.converted = False	
		return Sd(10, 1, 1) + Sa(10,1)