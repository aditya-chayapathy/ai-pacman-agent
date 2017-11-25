# keyboardAgents.py
# -----------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from game import Agent
from game import Directions
import random
import operator

class CompetitiveAgent(Agent):
    WEST_KEY  = 'a'
    EAST_KEY  = 'd'
    NORTH_KEY = 'w'
    SOUTH_KEY = 's'
    STOP_KEY = 'q'

    def __init__( self, index = 0 ):

        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []

    # This method is required and should return one of the 5 actions
    # Valid actions are Directions.NORTH, SOUTH, EAST, WEST and STOP
    # Appart from this restriction, you can do what ever you want in
    # thismethod to achieve your goals!
    def getAction(self, state):
        legalActions = state.getLegalActions(self.index)
        actionDict = {}
        for action in legalActions:
            actionDict[action] = self.evaluate_move(action, state)

        sortedDict = sorted(actionDict.items(), key=operator.itemgetter(1), reverse=True)

        return sortedDict[0][0]

    def evaluate_move(self, action, state):
        nextState = state.generateSuccessor(0, action)

        return 1

    # This is a simple helper, that can be ignored
    def getMove(self, legal):
        move = Directions.STOP
        if   (self.WEST_KEY in self.keys or 'Left' in self.keys) and Directions.WEST in legal:  move = Directions.WEST
        if   (self.EAST_KEY in self.keys or 'Right' in self.keys) and Directions.EAST in legal: move = Directions.EAST
        if   (self.NORTH_KEY in self.keys or 'Up' in self.keys) and Directions.NORTH in legal:   move = Directions.NORTH
        if   (self.SOUTH_KEY in self.keys or 'Down' in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
        return move
