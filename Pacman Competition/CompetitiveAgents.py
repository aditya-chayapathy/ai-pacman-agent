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
import util

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
        self.capsulesAvailable = set()

    # This method is required and should return one of the 5 actions
    # Valid actions are Directions.NORTH, SOUTH, EAST, WEST and STOP
    # Appart from this restriction, you can do what ever you want in
    # thismethod to achieve your goals!
    def getAction(self, state):
        legalActions = state.getLegalActions(self.index)
        pacmanPosition = state.getPacmanPosition()
        actionDict = {}
        for action in legalActions:
            actionDict[action] = self.evaluateAction(action, state)

        sortedDict = sorted(actionDict.items(), key=operator.itemgetter(1), reverse=True)
        bestAction = sortedDict[0][0]

        if pacmanPosition in self.capsulesAvailable:
            self.capsulesAvailable.remove(pacmanPosition)

        return bestAction

    def getNearestCapsule(self, pacmanPosition, capsulePositions, state):
        minDist = float("inf")
        minPos = capsulePositions[0]
        for position in capsulePositions:
            dist = self.mazeDistance(pacmanPosition, position, state)
            if minDist == float("inf") or dist <= minDist:
                minDist = dist
                minPos = position

        return minPos, minDist

    def evaluateAction(self, action, state):
        nextState = state.generateSuccessor(0, action)

        capsulePositions = nextState.getCapsules()
        isCapsuleAround = True if len(capsulePositions) > 0 else False
        pacmanPosition = nextState.getPacmanPosition()
        nearestCapsule, distanceToNearestCapsule = None, None
        if isCapsuleAround:
            nearestCapsule, distanceToNearestCapsule = self.getNearestCapsule(pacmanPosition, capsulePositions, nextState)
        for capsule in capsulePositions:
            self.capsulesAvailable.add(capsule)
        numberOfAgents = nextState.getNumAgents()
        isOpponentVisible = True if numberOfAgents > 1 else False
        pacmanAngryTimer = nextState.data.agentStates[0].angryTimer
        isPacmanAngry = True if pacmanAngryTimer > 0 else False
        actionValue = 0

        if isOpponentVisible:
            opponentPosition = nextState.getGhostPosition(1)
            opponentAngryTimer = nextState.data.agentStates[1].angryTimer
            isOpponentAngry = True if opponentAngryTimer > 0 else False
            pacmanOpponentDist = self.mazeDistance(pacmanPosition, opponentPosition, nextState)

            if isOpponentAngry and not isPacmanAngry:
                if action == Directions.STOP:
                    actionValue -= 1000
                if isCapsuleAround:
                    if distanceToNearestCapsule < pacmanOpponentDist:
                        actionValue -= (10 * distanceToNearestCapsule + pacmanOpponentDist)
                    else:
                        actionValue += (10 * pacmanOpponentDist + distanceToNearestCapsule)
                else:
                    actionValue += pacmanOpponentDist
            elif isPacmanAngry and not isOpponentAngry:
                actionValue -= pacmanOpponentDist
            elif isOpponentAngry and isPacmanAngry:
                if isCapsuleAround:
                    if distanceToNearestCapsule != 1:
                        actionValue -= distanceToNearestCapsule
                    else:
                        actionValue += 1000
                else:
                    actionValue -= pacmanOpponentDist
            elif not isPacmanAngry and not isOpponentAngry:
                if isCapsuleAround:
                    actionValue -= (10 * distanceToNearestCapsule)
                else:
                    actionValue -= pacmanOpponentDist

                if action == Directions.STOP:
                    actionValue -= 100
        else:
            if len(self.capsulesAvailable) != 0:
                nearestVisitedCapsule, distanceToNearestVisitedCapsule = self.getNearestCapsule(pacmanPosition, list(self.capsulesAvailable), nextState)
                actionValue -= distanceToNearestVisitedCapsule
            else:
                actionValue = random.randint(1, 100)
                if action == Directions.STOP:
                    actionValue -= 100

        return actionValue

    # This is a simple helper, that can be ignored
    def getMove(self, legal):
        move = Directions.STOP
        if   (self.WEST_KEY in self.keys or 'Left' in self.keys) and Directions.WEST in legal:  move = Directions.WEST
        if   (self.EAST_KEY in self.keys or 'Right' in self.keys) and Directions.EAST in legal: move = Directions.EAST
        if   (self.NORTH_KEY in self.keys or 'Up' in self.keys) and Directions.NORTH in legal:   move = Directions.NORTH
        if   (self.SOUTH_KEY in self.keys or 'Down' in self.keys) and Directions.SOUTH in legal: move = Directions.SOUTH
        return move

    def mazeDistance(self, point1, point2, gameState):
        """
        Returns the maze distance between any two points, using the search functions
        you have already built. The gameState can be any game state -- Pacman's
        position in that state is ignored.

        Example usage: mazeDistance( (2,4), (5,6), gameState)

        This might be a useful helper function for your ApproximateSearchAgent.
        """
        x1, y1 = point1
        x2, y2 = point2
        walls = gameState.getWalls()
        assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
        assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
        prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
        return len(self.breadthFirstSearch(prob))

    def breadthFirstSearch(self, problem):
        """Search the shallowest nodes in the search tree first."""
        "*** YOUR CODE HERE ***"
        fringe = util.Queue()  # Fringe (Queue) to store the nodes along with their paths
        visited_nodes = set()  # A set to maintain all the visited nodes
        fringe.push((problem.getStartState(), []))  # Pushing (Node, [Path from start-node till 'Node']) to the fringe
        while True:
            popped_element = fringe.pop()
            node = popped_element[0]
            path_till_node = popped_element[1]
            if problem.isGoalState(node):  # Exit on encountering goal node
                break
            else:
                if node not in visited_nodes:  # Skipping already visited nodes
                    visited_nodes.add(node)  # Adding newly encountered nodes to the set of visited nodes
                    successors = problem.getSuccessors(node)
                    for successor in successors:
                        child_node = successor[0]
                        child_path = successor[1]
                        full_path = path_till_node + [child_path]  # Computing path of child node from start node
                        fringe.push((child_node, full_path))  # Pushing ('Child Node',[Full Path]) to the fringe

        return path_till_node


class PositionSearchProblem(object):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1,1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print 'Warning: this does not look like a regular search maze'

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        # if isGoal and self.visualize:
        #     self._visitedlist.append(state)
        #     import __main__
        #     if '_display' in dir(__main__):
        #         if 'drawExpandedCells' in dir(__main__._display): #@UndefinedVariable
        #             __main__._display.drawExpandedCells(self._visitedlist) #@UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1 # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x,y= self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost

class Actions:
    """
    A collection of static methods for manipulating move actions.
    """
    # Directions
    _directions = {Directions.NORTH: (0, 1),
                   Directions.SOUTH: (0, -1),
                   Directions.EAST:  (1, 0),
                   Directions.WEST:  (-1, 0),
                   Directions.STOP:  (0, 0)}

    _directionsAsList = _directions.items()

    TOLERANCE = .001

    def reverseDirection(action):
        if action == Directions.NORTH:
            return Directions.SOUTH
        if action == Directions.SOUTH:
            return Directions.NORTH
        if action == Directions.EAST:
            return Directions.WEST
        if action == Directions.WEST:
            return Directions.EAST
        return action
    reverseDirection = staticmethod(reverseDirection)

    def vectorToDirection(vector):
        dx, dy = vector
        if dy > 0:
            return Directions.NORTH
        if dy < 0:
            return Directions.SOUTH
        if dx < 0:
            return Directions.WEST
        if dx > 0:
            return Directions.EAST
        return Directions.STOP
    vectorToDirection = staticmethod(vectorToDirection)

    def directionToVector(direction, speed = 1.0):
        dx, dy =  Actions._directions[direction]
        return (dx * speed, dy * speed)
    directionToVector = staticmethod(directionToVector)

    def getPossibleActions(config, walls):
        possible = []
        x, y = config.pos
        x_int, y_int = int(x + 0.5), int(y + 0.5)

        # In between grid points, all agents must continue straight
        if (abs(x - x_int) + abs(y - y_int)  > Actions.TOLERANCE):
            return [config.getDirection()]

        for dir, vec in Actions._directionsAsList:
            dx, dy = vec
            next_y = y_int + dy
            next_x = x_int + dx
            if not walls[next_x][next_y]: possible.append(dir)

        return possible

    getPossibleActions = staticmethod(getPossibleActions)

    def getLegalNeighbors(position, walls):
        x,y = position
        x_int, y_int = int(x + 0.5), int(y + 0.5)
        neighbors = []
        for dir, vec in Actions._directionsAsList:
            dx, dy = vec
            next_x = x_int + dx
            if next_x < 0 or next_x == walls.width: continue
            next_y = y_int + dy
            if next_y < 0 or next_y == walls.height: continue
            if not walls[next_x][next_y]: neighbors.append((next_x, next_y))
        return neighbors
    getLegalNeighbors = staticmethod(getLegalNeighbors)

    def getSuccessor(position, action):
        dx, dy = Actions.directionToVector(action)
        x, y = position
        return (x + dx, y + dy)
    getSuccessor = staticmethod(getSuccessor)
