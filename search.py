# search.py
# ---------
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


# search.py
# ---------
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import sys
import copy
import pdb

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def goalTest(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
        Given a state, returns available actions.
        Returns a list of actions
        """        
        util.raiseNotDefined()

    def getResult(self, state, action):
        """
        Given a state and an action, returns resulting state.
        """
        util.raiseNotDefined()

    def getCost(self, state, action):
        """
        Given a state and an action, returns step cost, which is the incremental cost 
        of moving to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

class Node:

    def __init__(self, problem, parent=None, action=None):

        self.state = problem.getStartState()
        self.pathCost = 0
        self.depth = 0

        self.parent = parent
        self.action = action

        if parent:
            self.state = problem.getResult(parent.getState(), action)
            self.pathCost = parent.getPathCost() + problem.getCost(parent.getState(), self.action)
            self.depth = parent.getDepth() + 1


    def __repr__(self):
        return "<Node %s>" % (self.state,)

    def __eq__(self, other):
        return self.state == other.state

    def getState(self):
        return self.state
    
    def getParent(self):
        return self.parent
    
    def getAction(self):
        return self.action

    def getPathCost(self):
        return self.pathCost
    
    def getDepth(self):
        return self.depth
    
    def path(self, problem):
        root = problem.getStartState()
        node = self
        actions = []

        while node.getState() != root:
            action = node.getAction()
            actions.insert(0, action)
            node = node.getParent()
        return actions

    def expand(self, problem):
        """
        A helper function to expand a given node get all the possible actions and their
        results as a list
        """
        actions = problem.getActions(self.state)
        children = [Node(problem, self, action) for action in actions]
        return children

class myQueue(util.Queue):

    def __contains__(self, item):
        return item in self.list

class myStack(util.Stack):
    def __contains__(self, item):
        return item in self.list


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.

    You are not required to implement this, but you may find it useful for Q5.
    """
    "*** YOUR CODE HERE ***"
    frontier = myQueue()
    explored = []
    start = Node(problem)

    frontier.push(start)   
    while not frontier.isEmpty():
        node = frontier.pop()
        explored.append(node)
        for child in node.expand(problem):
            if child not in explored and child not in frontier:
                if problem.goalTest(child.getState()):
                    return child.path(problem)
                frontier.push(child)
    return 'failure'

def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth.

    Begin with a depth of 1 and increment depth by 1 at every step.
    """
    "*** YOUR CODE HERE ***"

    def depthLimitedSearch(problem, limit):
        frontier = myStack()
        explored = []
        start = Node(problem)

        frontier.push(start)
        while not frontier.isEmpty():
            node = frontier.pop()
            explored.append(node)
            #pdb.set_trace()
            #if node.getDepth() == limit:
            #return 'cutoff'
            for child in node.expand(problem):
                if child not in explored and child not in frontier:
                    if problem.goalTest(child.getState()):
                        return child.path(problem)
                    if child.getDepth() < limit:
                        frontier.push(child)
        return 'cutoff'

    depth = 1
    while True:
        result = depthLimitedSearch(problem, depth)
        depth = depth + 1
        if result != 'cutoff':
            return result
            #print "Calculating on depth" + str(depth)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    start = Node(problem)
    frontier = util.PriorityQueueWithFunction(lambda node: 
                                              node.getPathCost() + 
                                              heuristic(node.getState(), problem))
    explored = []

    frontier.push(start)
    while True:
        if frontier.isEmpty():
            return 'failure'
        node = frontier.pop()
        #print "On node " + str(node)
        if problem.goalTest(node.getState()):
            path = node.path(problem)
            return path
        if node.getState() not in explored:
            explored.append(node.getState())
            for child in node.expand(problem):
                if child.getState() not in explored:
                    frontier.push(child)

# Abbreviations
bfs = breadthFirstSearch
astar = aStarSearch
ids = iterativeDeepeningSearch
