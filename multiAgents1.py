# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util, sys
import pdb
from search import Node

from game import Agent

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as a guide.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        # if ghost on you, worst neg number
        # if move takes you to 2 within ghost, pretty shitty neg number
        # did your move bring you closer or farther from the ghost
        # if no food left on board

        # best thing is to eat piece of food
        # moving toward the food
        # moving away from the food

        newCapsules = successorGameState.getCapsules()
        newScore = successorGameState.getScore()

        # Measure the delta in score between this state and the child
        # Closer to ghost is bad
        # Closer to food is good
        # Scared timer > 0 flips closer to ghost to good

        newFoodList = newFood.asList()
        newDistToFood = [util.manhattanDistance(newPos, food) for food in newFoodList]
        
        newGhostTuple = [(ghost.getPosition(), ghost.getDirection()) for ghost in newGhostStates]

        newDistToGhost = [(util.manhattanDistance(newPos, ghostPos), ghostDir)
                           for ghostPos, ghostDir in newGhostTuple]

        if newDistToFood == []:
            foodScore = sys.maxint
        #elif len(newDistToFood) <= 3:
            #pdb.set_trace()
        else:
            closestFood = float(min(newDistToFood))
            foodScore = 1/closestFood

        closestGhost = min(newDistToGhost, key=lambda tup: tup[0])
        if closestGhost[0] < 1:
            ghostScore = sys.maxint
        else:
            ghostScore = 1/closestGhost[0]
        
        # stops when nothing near and also doesn't take into account capsules
        # issue is that when you move and food is far away you're on a plateau
        # so there's no way to get out unless ghost bumps you randomly in the
        # right direction
        evalScore = foodScore - ghostScore

        print(foodScore)
        print(ghostScore)
        print(evalScore)
        print(newScore)
        #pdb.set_trace()
        return successorGameState.getScore() + evalScore

        #newGhostDir = [ghost.getDirection() for ghost in newGhostStates]
        #avgFood = reduce(lambda x, y: x + y, newDistToFood) / len(newDistToFood)
        #avgGhost = reduce(lambda x, y: x + y, newDistToGhost) / len(newDistToGhost)


def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent & AlphaBetaPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 7)
    """

    def getAction(self, gameState):
        """
          Returns the minimax action from the current gameState using self.depth
          and self.evaluationFunction.

          Here are some method calls that might be useful when implementing minimax.

          gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

          gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

          gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        "*** YOUR CODE HERE ***"
        numAgents = gameState.getNumAgents()

        # Showing no legal actions
        
        def minimax(gameState):
            legalActions = gameState.getLegalActions(self.index)
            vals = [(action, min_value(gameState.generateSuccessor(self.index, action), 
                                       1, self.depth)) for action in legalActions]
            max_val = max(vals, key=lambda tup: tup[1])
            action = max_val[0]
            return action
            
        def max_value(gameState, depth):
            if depth == 0:
                return self.evaluationFunction(gameState)
            v = -sys.maxint-1
            legalActions = gameState.getLegalActions(self.index)
            if legalActions == []:
                return self.evaluationFunction(gameState)
            for action in legalActions:
                v = max(v, min_value(gameState.generateSuccessor(self.index, action), 
                                     1, depth))
            return v
        def min_value(gameState, ghost, depth):
            if depth == 0:
                return self.evaluationFunction(gameState)
            v = sys.maxint
            legalActions = gameState.getLegalActions(ghost)
            if legalActions == []:
                return self.evaluationFunction(gameState)
            for action in legalActions:
                if ghost < numAgents-1:
                    v = min(v, min_value(gameState.generateSuccessor(ghost, action), 
                                         ghost+1, depth))                    
                else:
                    v = min(v, max_value(gameState.generateSuccessor(ghost, action), 
                                         depth-1))
            return v
        return minimax(gameState)


class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 8)
    """

    def getAction(self, gameState):
        """
          Returns the expectimax action using self.depth and self.evaluationFunction

          All ghosts should be modeled as choosing uniformly at random from their
          legal moves.
        """
        "*** YOUR CODE HERE ***"
        numAgents = gameState.getNumAgents()

        # Showing no legal actions
        
        def expectimax(gameState):
            legalActions = gameState.getLegalActions(self.index)
            vals = [(action, expect_value(gameState.generateSuccessor(self.index, action
), 1, self.depth)) for action in legalActions]
            max_val = max(vals, key=lambda tup: tup[1])
            action = max_val[0]
            return action
            
        def max_value(gameState, depth):
            if depth == 0:
                return self.evaluationFunction(gameState)
            v = -sys.maxint-1
            legalActions = gameState.getLegalActions(self.index)
            if legalActions == []:
                return self.evaluationFunction(gameState)
            for action in legalActions:
                v = max(v, expect_value(gameState.generateSuccessor(self.index, action), 1, depth))
            return v
        def expect_value(gameState, ghost, depth):
            if depth == 0:
                return self.evaluationFunction(gameState)
            legalActions = gameState.getLegalActions(ghost)
            if legalActions == []:
                return self.evaluationFunction(gameState)
            v = 0
            prob = 1/float(len(legalActions))
            for action in legalActions:
                if ghost < numAgents-1:
                    v = v + prob*expect_value(gameState.generateSuccessor(ghost, action)
                                              , ghost+1, depth)
                else:
                    v = v + prob*max_value(gameState.generateSuccessor(ghost, action),
                                           depth-1)
            return v
        return expectimax(gameState)

def betterEvaluationFunction(currentGameState):
    """
      Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
      evaluation function (question 9).

      DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction

